#include "hardware/hikcamera/hikcamera.hpp"
#include "hardware/usbcamera/usbcamera.hpp"
#include "tools/calibration/calibration.hpp"
#include "tools/config/config.hpp"
#include "tools/debug/debug.hpp"
#include "tools/exiter/exiter.hpp"
#include "tools/time/time.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{

// 控制台单键输入,主循环里轮询 's' 触发一次保存。非终端(管道/重定向)下禁用
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

// 内参标定配置
struct CalibrationConfig
{
    std::string                     camera{"usbcamera"}; // usbcamera | hikcamera
    tools::calibration::BoardConfig board;
    int                             samples{100}; // 需要的有效样本数
    std::string                     output{"apps/calibration/result.yaml"};
};

CalibrationConfig load_calibration_config(const tools::config::Config &config)
{
    CalibrationConfig cfg;
    cfg.camera            = config.value<std::string>("calibration.camera", "usbcamera");
    cfg.board.rows        = config.value<int>("calibration.board.rows", 11);
    cfg.board.cols        = config.value<int>("calibration.board.cols", 8);
    cfg.board.square_size = config.value<double>("calibration.board.square_size", 0.0181);
    cfg.samples           = config.value<int>("calibration.samples", 100);
    cfg.output            = config.value<std::string>("calibration.output", "apps/calibration/result.yaml");
    return cfg;
}

// 相对路径按仓库根解析,这样从任意工作目录启动都能找到配置文件。
std::string absolute_path(const std::string &path)
{
    if (std::filesystem::path(path).is_absolute())
    {
        return path;
    }
    return (std::filesystem::path(RM_PROJECT_ROOT) / path).string();
}

struct CameraFrame
{
    cv::Mat                image;
    tools::time::TimePoint timestamp;
};

std::string format_list(const std::vector<double> &values)
{
    std::ostringstream out;
    out << '[';
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        if (i != 0)
        {
            out << ", ";
        }
        out << std::setprecision(10) << values[i];
    }
    out << ']';
    return out.str();
}

// 写内参标定结果:camera_matrix + distort_coeff。
void write_result(const std::string &path, const cv::Mat &camera_matrix, const cv::Mat &distort_coeff)
{
    std::ofstream out(path);
    if (!out)
    {
        throw std::runtime_error("cannot write calibration result: " + path);
    }

    const cv::Mat       k = camera_matrix.reshape(1, 1);
    std::vector<double> k_values;
    k_values.reserve(9);
    for (int i = 0; i < 9; ++i)
    {
        k_values.push_back(k.at<double>(0, i));
    }
    std::vector<double> d_values;
    d_values.reserve(distort_coeff.total());
    for (int i = 0; i < static_cast<int>(distort_coeff.total()); ++i)
    {
        d_values.push_back(distort_coeff.at<double>(i));
    }

    out << "# calibration result \n";
    out << "camera_matrix: " << format_list(k_values) << "\n";
    out << "distort_coeff: " << format_list(d_values) << "\n";
}

// 把当前画面(带角点)发到 Rerun;失败只提示,不影响标定。
// 径向畸变必须单调(OpenCV 文档原话:估计出的参数非单调就是标定失败)。
// f(r) = 1 + k1 r^2 + k2 r^4 + k3 r^6,看 f'(r) = 2k1 r + 4k2 r^3 + 6k3 r^5 在成像范围内是否变号。
[[nodiscard]] bool distortion_monotonic(const cv::Mat &distort_coeff, double max_radius)
{
    const double k1 = distort_coeff.at<double>(0);
    const double k2 = distort_coeff.total() > 1 ? distort_coeff.at<double>(1) : 0.0;
    const double k3 = distort_coeff.total() > 4 ? distort_coeff.at<double>(4) : 0.0;

    int previous = 0;
    for (int i = 1; i <= 100; ++i)
    {
        const double r     = max_radius * static_cast<double>(i) / 100.0;
        const double slope = (2.0 * k1 * r) + (4.0 * k2 * r * r * r) + (6.0 * k3 * r * r * r * r * r);
        const int    sign  = slope >= 0.0 ? 1 : -1;
        if (previous != 0 && sign != previous)
        {
            return false;
        }
        previous = sign;
    }
    return true;
}

// 发一张已经画好的图到 Rerun(JPEG,比原始 BGR 小一两个数量级)。
void push_jpeg(tools::debug::Sink &sink, const cv::Mat &vis)
{
    if (!sink.active() || vis.empty())
    {
        return;
    }

    try
    {
        std::vector<std::uint8_t> jpeg;
        cv::imencode(".jpg", vis, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80});
        sink.image("calibration/image", std::move(jpeg));
    }
    catch (const std::exception &error)
    {
        tools::debug::log(tools::debug::Level::warn, std::string("skip calibration image: ") + error.what(), "calibration");
    }
}

void push_image(tools::debug::Sink &sink, const cv::Mat &image, const std::vector<cv::Point2f> &corners, const cv::Size &board, bool found,
                const std::string &label)
{
    if (!sink.active() || image.empty())
    {
        return;
    }

    cv::Mat vis = image.clone();
    if (found)
    {
        cv::drawChessboardCorners(vis, board, corners, true);
    }
    cv::putText(vis, label, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 2);
    push_jpeg(sink, vis);
}

} // namespace

int main(int argc, char **argv)
try
{
    (void)argc;
    (void)argv;

    const tools::config::Config config(RM_CONFIG_PATH);
    const CalibrationConfig     cfg    = load_calibration_config(config);
    const std::string           output = absolute_path(cfg.output);
    tools::install_exit_handler();
    cv::setNumThreads(1);
    tools::debug::Sink sink("rm_vision.calibration");

    // 相机
    std::unique_ptr<hardware::hikcamera::HikCamera> hik;
    std::unique_ptr<hardware::usbcamera::UsbCamera> usb;
    if (cfg.camera == "hikcamera")
    {
        hik = std::make_unique<hardware::hikcamera::HikCamera>(hardware::hikcamera::load_hikcamera_config(config));
    }
    else if (cfg.camera == "usbcamera")
    {
        usb = std::make_unique<hardware::usbcamera::UsbCamera>(hardware::usbcamera::load_usbcamera_config(config));
    }
    else
    {
        throw std::runtime_error("calibration.camera must be usbcamera/hikcamera, got " + cfg.camera);
    }

    const auto grab = [&]() -> std::optional<CameraFrame> {
        if (hik)
        {
            hardware::hikcamera::HikFrame frame;
            if (hik->frames().try_pop(frame) && !frame.image.empty())
            {
                return CameraFrame{frame.image, frame.timestamp};
            }
        }
        else
        {
            hardware::usbcamera::UsbFrame frame;
            if (usb->frames().try_pop(frame) && !frame.image.empty())
            {
                return CameraFrame{frame.image, frame.timestamp};
            }
        }
        return std::nullopt;
    };

    tools::debug::log(tools::debug::Level::info,
                      "calibration started: camera=" + cfg.camera + " board=" + std::to_string(cfg.board.size().width) + "x" +
                          std::to_string(cfg.board.size().height) + " square=" + std::to_string(cfg.board.square_size) +
                          "m samples=" + std::to_string(cfg.samples),
                      "calibration");

    const std::vector<cv::Point3f> object_points = cfg.board.object_points();
    const int                      flags         = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
    const cv::TermCriteria         criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001);

    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size                              image_size;

    tools::time::TimePoint last_push;
    bool                   has_last_push = false;
    std::int64_t           debug_frame   = 0;

    // 手动触发:控制台按 s 保存当前帧。
    ConsoleKey console;
    tools::debug::log(tools::debug::Level::info, "press 's' in the console to capture the current frame", "calibration");

    while (!tools::should_exit())
    {
        if (image_points.size() >= static_cast<std::size_t>(cfg.samples))
        {
            break;
        }

        const std::optional<CameraFrame> frame = grab();
        if (!frame)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        image_size = frame->image.size();

        cv::Mat gray;
        cv::cvtColor(frame->image, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        const bool               found = cv::findChessboardCorners(gray, cfg.board.size(), corners, flags);
        if (found)
        {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        }

        const tools::time::TimePoint now = tools::time::now();
        if (!has_last_push || now - last_push > std::chrono::milliseconds(200))
        {
            // 记到帧序号 + 时间轴上
            sink.set_frame(++debug_frame);
            sink.set_time("time", tools::time::since_base(frame->timestamp));
            push_image(sink, frame->image, corners, cfg.board.size(), found,
                       "captured " + std::to_string(image_points.size()) + "/" + std::to_string(cfg.samples) + "  (s=capture)");
            last_push     = now;
            has_last_push = true;
        }

        // 手动触发:按 s 才尝试保存当前帧。
        if (!console.save_requested())
        {
            continue;
        }

        // 保存前先判断当前帧是不是有效棋盘格数据,无效就不采。
        if (!found)
        {
            tools::debug::log(tools::debug::Level::warn, "ignore save: no chessboard detected", "calibration");
            continue;
        }
        if (static_cast<int>(corners.size()) != cfg.board.rows * cfg.board.cols)
        {
            tools::debug::log(tools::debug::Level::warn,
                              "ignore save: incomplete chessboard (" + std::to_string(corners.size()) + "/" +
                                  std::to_string(cfg.board.rows * cfg.board.cols) + " corners)",
                              "calibration");
            continue;
        }

        image_points.push_back(corners);
        tools::debug::log(tools::debug::Level::info, "captured " + std::to_string(image_points.size()) + "/" + std::to_string(cfg.samples),
                          "calibration");
    }

    if (image_points.size() < 4)
    {
        tools::debug::log(tools::debug::Level::error, "not enough intrinsic samples: " + std::to_string(image_points.size()), "calibration");
        return 1;
    }
    const tools::calibration::IntrinsicResult result = tools::calibration::calibrate_intrinsic(image_points, cfg.board, image_size);
    write_result(output, result.camera_matrix, result.distort_coeff);
    tools::debug::log(tools::debug::Level::info,
                      "intrinsics saved: " + output + " fx=" + std::to_string(result.camera_matrix.at<double>(0, 0)) + " fy=" +
                          std::to_string(result.camera_matrix.at<double>(1, 1)) + " reprojection=" + std::to_string(result.reprojection_error) + "px",
                      "calibration");

    // 自检一下畸变模型:参数虽然拟合上了,但可能只在画面中央成立。
    const double rel_x      = std::max(std::abs(result.camera_matrix.at<double>(0, 2)) / result.camera_matrix.at<double>(0, 0),
                                       std::abs(image_size.width - result.camera_matrix.at<double>(0, 2)) / result.camera_matrix.at<double>(0, 0));
    const double rel_y      = std::max(std::abs(result.camera_matrix.at<double>(1, 2)) / result.camera_matrix.at<double>(1, 1),
                                       std::abs(image_size.height - result.camera_matrix.at<double>(1, 2)) / result.camera_matrix.at<double>(1, 1));
    const double max_radius = std::sqrt((rel_x * rel_x) + (rel_y * rel_y));
    if (!distortion_monotonic(result.distort_coeff, max_radius))
    {
        tools::debug::log(tools::debug::Level::warn,
                          "distortion is not monotonic within the image, the model is unreliable: 常见原因是标定板只覆盖了画面中央;"
                          "重新采集时让棋盘格布满四角和边缘,并多换几个倾角",
                          "calibration");
    }

    // ---- 校验 ----
    // 上面那个 reprojection 是"对采集到的这几张图"的残差,数据分布不好时它会虚低(全都拍在画面
    // 中央就很好看),所以不算校验。这里拿实时新画面重新 PnP + 重投影算误差,并把去畸变画面发到
    // Rerun:误差小 + 棋盘格直线仍然是直的,才算标定可用。
    tools::debug::log(tools::debug::Level::info, "verify: keep the board in view; reprojection error and undistorted view go to Rerun",
                      "calibration");

    const cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(result.camera_matrix, result.distort_coeff, image_size, 0.0);
    cv::Mat       undistort_map1;
    cv::Mat       undistort_map2;
    cv::initUndistortRectifyMap(result.camera_matrix, result.distort_coeff, cv::Mat(), new_camera_matrix, image_size, CV_16SC2, undistort_map1,
                                undistort_map2);

    std::size_t            verify_count = 0;
    double                 verify_mean  = 0.0;
    double                 verify_max   = 0.0;
    tools::time::TimePoint last_report;
    bool                   has_last_report = false;

    while (!tools::should_exit())
    {
        const std::optional<CameraFrame> frame = grab();
        if (!frame)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(frame->image, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        const bool               found = cv::findChessboardCorners(gray, cfg.board.size(), corners, flags);
        const bool               valid = found && corners.size() == object_points.size();

        const tools::time::TimePoint now  = tools::time::now();
        const bool                   push = valid && (!has_last_push || now - last_push > std::chrono::milliseconds(200));
        if (push)
        {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // 用标定参数解出棋盘位姿,再投回图像,和检测到的角点比;重投影误差。
            cv::Vec3d rvec;
            cv::Vec3d tvec;
            cv::solvePnP(object_points, corners, result.camera_matrix, result.distort_coeff, rvec, tvec);
            std::vector<cv::Point2f> reprojected;
            cv::projectPoints(object_points, rvec, tvec, result.camera_matrix, result.distort_coeff, reprojected);

            double error_sum = 0.0;
            double error_max = 0.0;
            for (std::size_t i = 0; i < corners.size(); ++i)
            {
                const double error = cv::norm(corners[i] - reprojected[i]);
                error_sum += error * error;
                error_max = std::max(error_max, error);
            }
            const double rms = std::sqrt(error_sum / static_cast<double>(corners.size()));
            ++verify_count;
            verify_mean = verify_mean + ((rms - verify_mean) / static_cast<double>(verify_count));
            verify_max  = std::max(verify_max, rms);

            // 画在去畸变图上:两组点要过同样的去畸变才能和画面重合。
            cv::Mat undistorted;
            cv::remap(frame->image, undistorted, undistort_map1, undistort_map2, cv::INTER_LINEAR);
            std::vector<cv::Point2f> detected_undistorted;
            std::vector<cv::Point2f> reprojected_undistorted;
            cv::undistortPoints(corners, detected_undistorted, result.camera_matrix, result.distort_coeff, cv::noArray(), new_camera_matrix);
            cv::undistortPoints(reprojected, reprojected_undistorted, result.camera_matrix, result.distort_coeff, cv::noArray(), new_camera_matrix);
            for (std::size_t i = 0; i < detected_undistorted.size(); ++i)
            {
                // 绿圈=检测到的角点,红点=用标定参数重投影出的角点,越重合标定越准。
                cv::circle(undistorted, detected_undistorted[i], 6, cv::Scalar(0, 255, 0), 2);
                cv::circle(undistorted, reprojected_undistorted[i], 3, cv::Scalar(0, 0, 255), -1);
            }
            cv::putText(undistorted, "verify rms=" + cv::format("%.2f", rms) + "px", cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2,
                        cv::Scalar(0, 255, 0), 2);

            sink.set_frame(++debug_frame);
            sink.set_time("time", tools::time::since_base(frame->timestamp));
            sink.data("calibration/reprojection_error", rms);
            push_jpeg(sink, undistorted);

            last_push     = now;
            has_last_push = true;
        }

        if (verify_count > 0 && (!has_last_report || now - last_report > std::chrono::seconds(1)))
        {
            tools::debug::log(tools::debug::Level::info,
                              "verify: mean=" + cv::format("%.3f", verify_mean) + "px max=" + cv::format("%.3f", verify_max) +
                                  "px n=" + std::to_string(verify_count) + (valid ? "" : " (board not visible)"),
                              "calibration");
            last_report     = now;
            has_last_report = true;
        }
    }

    return 0;
}
catch (const std::exception &error)
{
    try
    {
        tools::debug::log(tools::debug::Level::error, std::string("calibration exception: ") + error.what(), "calibration");
    }
    catch (...)
    {
        std::fputs("calibration exception (log failed)\n", stderr);
    }
    return 1;
}
