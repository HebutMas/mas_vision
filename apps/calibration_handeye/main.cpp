#include "hardware/hikcamera/hikcamera.hpp"
#include "hardware/serialport/serialport.hpp"
#include "hardware/usbcamera/usbcamera.hpp"
#include "tools/algorithm/math_tools.hpp"
#include "tools/calibration/calibration.hpp"
#include "tools/config/config.hpp"
#include "tools/debug/debug.hpp"
#include "tools/exiter/exiter.hpp"
#include "tools/time/time.hpp"

#include <Eigen/Geometry>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstddef>
#include <exception>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
// 控制台单键输入,主循环里轮询 's' 触发一次采集。非终端(管道/重定向)下禁用。
class ConsoleKey
{
  public:
    ConsoleKey()
    {
        if (::isatty(STDIN_FILENO) == 0 || ::tcgetattr(STDIN_FILENO, &saved_) != 0)
        {
            return;
        }
        termios raw = saved_;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
        {
            return;
        }
        saved_flags_ = ::fcntl(STDIN_FILENO, F_GETFL, 0);
        ::fcntl(STDIN_FILENO, F_SETFL, saved_flags_ | O_NONBLOCK);
        active_ = true;
    }

    ConsoleKey(const ConsoleKey &)            = delete;
    ConsoleKey &operator=(const ConsoleKey &) = delete;

    ~ConsoleKey()
    {
        if (!active_)
        {
            return;
        }
        ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
        ::fcntl(STDIN_FILENO, F_SETFL, saved_flags_);
    }

    // 读走当前所有待处理按键;返回是否按下了 's'/'S'。
    bool save_requested()
    {
        bool hit = false;
        char key = 0;
        while (::read(STDIN_FILENO, &key, 1) == 1)
        {
            if (key == 's' || key == 'S')
            {
                hit = true;
            }
        }
        return hit;
    }

  private:
    termios saved_{};
    int     saved_flags_{0};
    bool    active_{false};
};
using hardware::hikcamera::HikCamera;
using hardware::hikcamera::HikFrame;
using hardware::usbcamera::UsbCamera;
using hardware::usbcamera::UsbFrame;

// 手眼标定配置
struct HandEyeConfig
{
    std::string                     camera{"usbcamera"}; // usbcamera / hikcamera
    tools::calibration::BoardConfig board;               // 标定板(棋盘格)
    int                             samples{15};         // 需要多少组
    std::string                     output{"apps/calibration_handeye/result.yaml"};
    cv::Mat                         camera_matrix; // 内参 3x3 CV_64F
    cv::Mat                         distort_coeff; // 畸变 1xN CV_64F
};

// 读配置:标定参数 + 相机内参
HandEyeConfig load_hand_eye_config(const tools::config::Config &config)
{
    HandEyeConfig     out;
    const std::string t   = "calibration_handeye.";
    out.camera            = config.value(t + "camera", out.camera);
    out.board.rows        = config.value(t + "board.rows", out.board.rows);
    out.board.cols        = config.value(t + "board.cols", out.board.cols);
    out.board.square_size = config.value(t + "board.square_size", out.board.square_size);
    out.samples           = config.value(t + "samples", out.samples);
    out.output            = config.value(t + "output", out.output);

    const auto k      = config.require<std::vector<double>>("hikcamera.camera_matrix");
    auto       d      = config.value<std::vector<double>>("hikcamera.distort_coeffs", {});
    out.camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double *>(k.data())).clone();
    out.distort_coeff = cv::Mat(1, static_cast<int>(d.size()), CV_64F, const_cast<double *>(d.data())).clone();
    return out;
}

// 相对路径按仓库根解析,不依赖启动时的工作目录。
std::string absolute_path(const std::string &path)
{
    if (path.empty() || path.front() == '/')
    {
        return path;
    }
    return std::string(RM_PROJECT_ROOT) + "/" + path;
}

// 3x3 的 cv::Mat -> vector<double>(行优先),写 yaml 用。
std::vector<double> mat_to_list(const cv::Mat &mat)
{
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(mat.total()));
    for (int row = 0; row < mat.rows; ++row)
    {
        for (int col = 0; col < mat.cols; ++col)
        {
            values.push_back(mat.at<double>(row, col));
        }
    }
    return values;
}

std::string format_list(const std::vector<double> &values)
{
    std::ostringstream out;
    out << '[' << std::setprecision(10);
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        out << (i == 0 ? "" : ", ") << values[i];
    }
    out << ']';
    return out.str();
}

// 写标定结果(
void write_result(const std::string &path, const cv::Mat &r_camera2gimbal, const cv::Mat &t_camera2gimbal, double consistency_deg,
                  const Eigen::Vector3d &camera_ypr_deg)
{
    std::ofstream file(path);
    if (!file)
    {
        throw std::runtime_error("cannot write result file: " + path);
    }
    file << std::setprecision(10);
    file << "# Hand eye calibration result: camera -> gimbal\n";
    file << "# AX=BX consistency deviation: " << std::setprecision(3) << consistency_deg
         << " deg (the smaller the better; >1 means the sampled angles were too concentrated)\n";
    file << "# Camera offset from ideal mounting: yaw " << camera_ypr_deg[0] << " pitch " << camera_ypr_deg[1] << " roll " << camera_ypr_deg[2]
         << " deg\n";
    file << "R_camera2gimbal: " << format_list(mat_to_list(r_camera2gimbal)) << '\n';
    file << "t_camera2gimbal: " << format_list(mat_to_list(t_camera2gimbal)) << '\n';
}
} // namespace

// 手眼标定:固定标定板,每转到一个新的云台角度按 s 采集一组
// NOLINTNEXTLINE(bugprone-exception-escape): catch 内日志可能抛异常,进程直接退出即可。
int main(int argc, char **argv)
try
{
    (void)argc;
    (void)argv;
    const tools::config::Config config(RM_CONFIG_PATH);
    const HandEyeConfig         hand_eye    = load_hand_eye_config(config);
    const std::string           output_path = absolute_path(hand_eye.output);

    tools::install_exit_handler();

    // 相机
    std::unique_ptr<HikCamera> hik;
    std::unique_ptr<UsbCamera> usb;
    if (hand_eye.camera == "hikcamera")
    {
        hik = std::make_unique<HikCamera>(hardware::hikcamera::load_hikcamera_config(config));
    }
    else if (hand_eye.camera == "usbcamera")
    {
        usb = std::make_unique<UsbCamera>(hardware::usbcamera::load_usbcamera_config(config));
    }
    else
    {
        throw std::invalid_argument("unknown camera: " + hand_eye.camera);
    }

    // 云台姿态
    const hardware::serialport::SerialPort serial(config);

    const std::vector<cv::Point3f> object_points = hand_eye.board.object_points();
    const int                      flags         = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
    const cv::TermCriteria         criteria{cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001};

    std::vector<tools::calibration::HandEyeSample> samples;
    auto                                           last_tick = std::chrono::steady_clock::now();

    // 手动触发:控制台按 s 采集当前帧。每组尽量换一个明显不同的云台角度。
    ConsoleKey console;
    tools::debug::log(tools::debug::Level::info,
                      "hand eye calibration: keep the board clearly visible, press s at each new gimbal angle to capture one sample, " +
                          std::to_string(hand_eye.samples) + " samples needed");

    while (!tools::should_exit() && static_cast<int>(samples.size()) < hand_eye.samples)
    {
        // 图像
        cv::Mat                image;
        tools::time::TimePoint timestamp;
        if (hik)
        {
            HikFrame frame;
            if (!hik->frames().try_pop(frame) || frame.image.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            image     = frame.image;
            timestamp = frame.timestamp;
        }
        else
        {
            UsbFrame frame;
            if (!usb->frames().try_pop(frame) || frame.image.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            image     = frame.image;
            timestamp = frame.timestamp;
        }

        // 云台姿态
        const Eigen::Matrix3d r_gimbal2world = serial.quaternion_at(timestamp).toRotationMatrix().cast<double>();

        // 识别棋盘格
        cv::Mat                  gray;
        std::vector<cv::Point2f> corners;
        bool                     found = false;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        if (cv::findChessboardCorners(gray, hand_eye.board.size(), corners, flags))
        {
            cv::cornerSubPix(gray, corners, {11, 11}, {-1, -1}, criteria);
            found = true;
        }

        // 板没识别到时每秒提示一次
        const auto now = std::chrono::steady_clock::now();
        if (!found && now - last_tick > std::chrono::seconds(1))
        {
            last_tick = now;
            tools::debug::log(tools::debug::Level::warn, "waiting: no chessboard detected");
        }

        // 手动触发:按 s 采集当前帧
        if (!console.save_requested())
        {
            continue;
        }
        // 采集前先确认这帧是有效的完整棋盘格,无效就不采
        if (!found)
        {
            tools::debug::log(tools::debug::Level::warn, "ignore save: no chessboard detected");
            continue;
        }
        if (corners.size() != object_points.size())
        {
            tools::debug::log(tools::debug::Level::warn, "ignore save: incomplete chessboard");
            continue;
        }

        cv::Mat rvec;
        cv::Mat tvec;
        cv::solvePnP(object_points, corners, hand_eye.camera_matrix, hand_eye.distort_coeff, rvec, tvec);

        tools::calibration::HandEyeSample sample;
        cv::eigen2cv(r_gimbal2world, sample.r_gimbal2world);
        sample.t_gimbal2world = cv::Mat::zeros(3, 1, CV_64F); // 云台位置视为世界原点
        sample.rvec           = rvec;
        sample.tvec           = tvec;
        samples.push_back(sample);

        const Eigen::Vector3d ypr = tools::eulers(r_gimbal2world, 2, 1, 0) * 180.0 / M_PI;
        std::ostringstream    message;
        message << "captured " << samples.size() << '/' << hand_eye.samples << std::setprecision(1) << std::fixed << "  gimbal ypr=(" << ypr[0]
                << ", " << ypr[1] << ", " << ypr[2] << ") deg  dist=" << cv::norm(tvec) << " m";
        tools::debug::log(tools::debug::Level::info, message.str());
    }

    if (samples.size() < 3)
    {
        tools::debug::log(tools::debug::Level::error, "not enough samples: " + std::to_string(samples.size()));
        return 1;
    }
    if (static_cast<int>(samples.size()) < hand_eye.samples)
    {
        tools::debug::log(tools::debug::Level::warn, "early exit: only " + std::to_string(samples.size()) + " samples captured");
    }

    // 手眼标定:输入云台在世界系的姿态 + 板在相机系的位姿,输出相机到云台的变换。
    const tools::calibration::HandEyeResult hand_eye_result = tools::calibration::calibrate_hand_eye(samples);

    // 相机同理想安装的偏角，理想基:相机光轴与云台 x 重合
    Eigen::Matrix3d r_camera2gimbal_eigen;
    cv::cv2eigen(hand_eye_result.r_camera2gimbal, r_camera2gimbal_eigen);
    const Eigen::Matrix3d r_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
    const Eigen::Vector3d camera_ypr = tools::eulers(r_gimbal2ideal * r_camera2gimbal_eigen, 1, 0, 2) * 180.0 / M_PI;

    write_result(output_path, hand_eye_result.r_camera2gimbal, hand_eye_result.t_camera2gimbal, hand_eye_result.consistency_deg, camera_ypr);

    std::ostringstream summary;
    summary << std::setprecision(6) << std::fixed << "hand eye calibration done: samples=" << samples.size()
            << " consistency=" << hand_eye_result.consistency_deg << " deg  camera ypr=(" << camera_ypr[0] << ", " << camera_ypr[1] << ", "
            << camera_ypr[2] << ") deg  t=(" << hand_eye_result.t_camera2gimbal.at<double>(0) << ", " << hand_eye_result.t_camera2gimbal.at<double>(1)
            << ", " << hand_eye_result.t_camera2gimbal.at<double>(2) << ") m -> " << output_path;
    tools::debug::log(tools::debug::Level::info, summary.str());
    std::cout << summary.str() << '\n';

    return 0;
}
catch (const std::exception &error)
{
    try
    {
        tools::debug::log(tools::debug::Level::error, std::string("calibration_handeye exception: ") + error.what());
    }
    catch (...)
    {
        static_cast<void>(0);
    }
    return 1;
}
