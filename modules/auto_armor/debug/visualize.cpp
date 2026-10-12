#include "modules/auto_armor/debug/visualize.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace rm::armor
{
namespace
{

// Canvas调色板
[[nodiscard]] cv::Scalar shape_color() noexcept { return {127, 127, 127}; } // 灰:未锁定装甲板
[[nodiscard]] cv::Scalar locked_color() noexcept { return {0, 255, 0}; }    // 绿:当前锁定装甲板
[[nodiscard]] cv::Scalar arrow_color() noexcept { return {255, 255, 255}; } // 白:角点外指箭头
[[nodiscard]] cv::Scalar id_color() noexcept { return {0, 255, 255}; }      // 黄:id 标签
[[nodiscard]] cv::Scalar white_color() noexcept { return {255, 255, 255}; }
[[nodiscard]] cv::Scalar red_color() noexcept { return {0, 0, 255}; }

// 云台系(x 前,y 左,z 上) -> 相机系(OpenCV x 右,y 下,z 前)。
[[nodiscard]] Eigen::Vector3d gimbal_to_camera(const Eigen::Vector3d &gimbal) noexcept { return {-gimbal.y(), -gimbal.z(), gimbal.x()}; }

// 装甲板物点:本体系(x 前=法线,y 左,z 上),顺序 tl,tr,br,bl,单位米。
[[nodiscard]] std::vector<cv::Point3f> make_object_points(double width, double height)
{
    const auto hy = static_cast<float>(width) / 2.0F / 1000.0F;
    const auto hz = static_cast<float>(height) / 2.0F / 1000.0F;
    return {{0.0F, hy, hz}, {0.0F, -hy, hz}, {0.0F, -hy, -hz}, {0.0F, hy, -hz}};
}

[[nodiscard]] std::vector<cv::Point3f> object_points(const SolverConfig &config, Kind kind)
{
    const auto &geometry = config.geometry;
    const bool  large    = (kind == Kind::hero);
    return large ? make_object_points(geometry.large_width, geometry.large_height) : make_object_points(geometry.small_width, geometry.small_height);
}

// 文本绘制
void draw_text(cv::Mat &image, const std::string &text, cv::Point origin, const cv::Scalar &color, double scale, int thickness)
{
    cv::putText(image, text, origin + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), thickness + 2, cv::LINE_AA);
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
}

// 把一块装甲板3D 位姿重投影回像素,得到四个角点。
[[nodiscard]] std::vector<cv::Point2f> reproject_armor(const SolverConfig &config, const Eigen::Vector3d &xyz, double yaw, Kind kind)
{
    // 板倾角:机器人装甲 ~75°(法线抬 15°),前哨站板略俯。
    const double pitch = (kind == Kind::outpost ? -15.0 : 15.0) * M_PI / 180.0;
    // solver 的 yaw 是板位置方位角,法线方位角 = yaw + π;本体系 col0=法线,col1=左,col2=上。
    const double yaw_n = yaw + M_PI;
    const double c     = std::cos(yaw_n);
    const double s     = std::sin(yaw_n);
    const double cp    = std::cos(pitch);
    const double sp    = std::sin(pitch);

    Eigen::Matrix3d r_armor2gimbal;
    r_armor2gimbal.col(0) = Eigen::Vector3d(c * cp, s * cp, -sp);
    r_armor2gimbal.col(1) = Eigen::Vector3d(-s, c, 0.0);
    r_armor2gimbal.col(2) = Eigen::Vector3d(c * sp, s * sp, cp);

    Eigen::Matrix3d r_gimbal2camera;
    r_gimbal2camera << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0;

    const Eigen::Matrix3d r_armor2camera = r_gimbal2camera * r_armor2gimbal;

    cv::Mat rot;
    cv::eigen2cv(r_armor2camera, rot);
    cv::Mat rvec;
    cv::Rodrigues(rot, rvec);

    const Eigen::Vector3d t_armor2camera = gimbal_to_camera(xyz);
    const cv::Mat         tvec           = (cv::Mat_<double>(3, 1) << t_armor2camera.x(), t_armor2camera.y(), t_armor2camera.z());

    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points(config, kind), rvec, tvec, config.camera_matrix, config.distort_coeff, image_points);
    return image_points;
}

// 单个 3D 点投影到像素。
[[nodiscard]] cv::Point2f project(const SolverConfig &config, const Eigen::Vector3d &xyz)
{
    const Eigen::Vector3d          camera = gimbal_to_camera(xyz);
    const std::vector<cv::Point3f> obj{{
        static_cast<float>(camera.x()),
        static_cast<float>(camera.y()),
        static_cast<float>(camera.z()),
    }};

    std::vector<cv::Point2f> image;
    cv::projectPoints(obj, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), config.camera_matrix, config.distort_coeff, image);
    return image.front();
}

} // namespace

void draw(cv::Mat &image, const std::optional<track::TargetState> &state, const SolverConfig &solver)
{
    if (image.empty())
    {
        return;
    }

    // 字号/线宽随分辨率放大(基准 0.4 / 1,与 rmcs 一致)。
    const double font_scale = std::max(0.4, image.rows / 1080.0);
    const int    thickness  = 1 + (image.rows / 1080);
    const int    text_x     = cvRound(10.0 * font_scale);
    const int    text_y     = image.rows - cvRound(90.0 * font_scale);

    // 光学中心十字(红)。
    const auto  half = static_cast<float>(12.0 * font_scale);
    const float ox   = static_cast<float>(image.cols) / 2.0F;
    const float oy   = static_cast<float>(image.rows) / 2.0F;
    cv::line(image, {cvRound(ox - half), cvRound(oy)}, {cvRound(ox + half), cvRound(oy)}, red_color(), thickness, cv::LINE_AA);
    cv::line(image, {cvRound(ox), cvRound(oy - half)}, {cvRound(ox), cvRound(oy + half)}, red_color(), thickness, cv::LINE_AA);

    if (!state)
    {
        draw_text(image, "track: lost", {text_x, text_y}, red_color(), 0.6 * font_scale, thickness);
        return;
    }

    const int  dot_radius = 2 + (image.rows / 1080);
    const auto arrow_len  = static_cast<float>(5.0 * font_scale);

    // 每块预测板重投影回像素:灰框 + 角点圆点 + 角点外指白箭头 + id 标签;
    // 当前锁定的那块(last_id)用绿框并标注兵种/颜色/置信度。
    for (int id = 0; id < state->armor_num; ++id)
    {
        const auto reproj = reproject_armor(solver, state->armor_xyz(id), state->armor_yaw(id), state->kind);
        if (reproj.size() < 4)
        {
            continue;
        }

        const bool       locked     = (id == state->last_id);
        const cv::Scalar color      = locked ? locked_color() : shape_color();
        const int        line_width = locked ? thickness + 1 : thickness;

        cv::Point2f centroid{0.0F, 0.0F};
        for (const auto &corner : reproj)
        {
            centroid += corner;
        }
        centroid /= static_cast<float>(reproj.size());

        cv::Point2f top = reproj.front();
        for (std::size_t i = 0; i < reproj.size(); ++i)
        {
            cv::line(image, reproj[i], reproj[(i + 1) % reproj.size()], color, line_width, cv::LINE_AA);

            const auto &corner = reproj[i];
            cv::circle(image, corner, dot_radius, color, cv::FILLED, cv::LINE_AA);

            const cv::Point2f outward = corner - centroid;
            const auto        norm    = std::hypot(outward.x, outward.y);
            if (norm > 1.0E-6F)
            {
                cv::arrowedLine(image, corner + (outward * (arrow_len / norm)), corner, arrow_color(), thickness, cv::LINE_AA);
            }
            if (corner.y < top.y)
            {
                top = corner;
            }
        }

        const cv::Point label_origin{cvRound(top.x), cvRound(top.y - (15.0 * font_scale))};
        if (locked)
        {
            const std::string label =
                std::string(kind_name(state->kind)) + " " + color_name(state->color) + " " + cv::format("%.2f", state->confidence);
            draw_text(image, label, label_origin, locked_color(), 0.5 * font_scale, thickness);
        }
        else
        {
            draw_text(image, std::to_string(id), label_origin, id_color(), 0.4 * font_scale, thickness);
        }
    }

    // 整车中心(品红实心点)+ 0.2s 速度方向(青箭头)。
    const cv::Point2f center = project(solver, state->center);
    cv::circle(image, center, dot_radius + 2, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_AA);
    cv::arrowedLine(image, center, project(solver, state->center + (state->vel * 0.2)), cv::Scalar(255, 255, 0), thickness, cv::LINE_AA);

    // 左下角文本面板。
    const int step    = cvRound(22.0 * font_scale);
    int       panel_y = text_y;
    draw_text(image, cv::format("%.2fm", state->center.norm()), {text_x, panel_y}, white_color(), 0.6 * font_scale, thickness);
    panel_y += step;
    draw_text(image, state->tracking ? "TRACK" : "TEMP_LOST", {text_x, panel_y}, state->tracking ? red_color() : white_color(), 0.6 * font_scale,
              thickness);
    panel_y += step;
    draw_text(image, cv::format("id=%d w=%.2f r=%.2f", state->last_id, state->w, state->r), {text_x, panel_y}, white_color(), 0.6 * font_scale,
              thickness);
}

} // namespace rm::armor
