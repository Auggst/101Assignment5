#include "Triangle.hpp"
#include "Camera.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
        Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

        Eigen::Matrix4f translate;
        translate << 1, 0, 0, -eye_pos[0],
            0, 1, 0, -eye_pos[1],
            0, 0, 1, -eye_pos[2],
            0, 0, 0, 1;

        view = translate * view;

        return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

        Eigen::Matrix4f rotate;
        float angle = rotation_angle / 180 * MY_PI;
        rotate << cos(angle), -sin(angle), 0, 0,
            sin(angle), cos(angle), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        model = rotate * model;

        return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar, bool isOrth)
{

        Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

        float l, r, b, t, n, f;
        n = zNear;
        f = zFar;
        b = zNear * tan((eye_fov / 2) / 180 * MY_PI);
        t = -b;
        l = b * aspect_ratio;
        r = -l;
        Eigen::Matrix4f orth;
        orth << 2 / (r - l), 0, 0, -(r + l) / (r - l),
            0, 2 / (t - b), 0, -(t + b) / (t - b),
            0, 0, 2 / (n - f), -(n + f) / (n - f),
            0, 0, 0, 1;
        if (isOrth)
        {
                projection = orth * projection;
        }
        else
        {
                Eigen::Matrix4f persp2orth;
                persp2orth << n, 0, 0, 0,
                    0, n, 0, 0,
                    0, 0, n + f, -f * n,
                    0, 0, 1, 0;
                projection = orth * persp2orth * projection;
        }

        return projection;
}


int main(int argc, const char **argv)
{
        float angle = 0;
        bool command_line = false;
        std::string filename = "output.png";
        bool isoth = true;
        MSAA msaa = MSAAno;
        if (argc >= 3)
        {
                command_line = true;
                angle = std::stof(argv[2]); // -r by default
                if (argc == 4)
                {
                        filename = std::string(argv[3]);
                }
                else if (argc == 6) //-p 透视模式
                {
                        filename = std::string(argv[3]);
                        if (std::string(argv[5]) == "false")
                                isoth = false;
                }
                else if (argc == 8) //-msaa 反走样
                {
                        filename = std::string(argv[3]);
                        if (std::string(argv[5]) == "false")
                                isoth = false;
                        int val = std::stoi(argv[7]);
                        switch (val)
                        {
                        case 2:
                                msaa = MSAA2x;
                                break;
                        case 4:
                                msaa = MSAA4x;
                                break;
                        case 8:
                                msaa = MSAA8x;
                                break;
                        default:
                                msaa = MSAAno;
                                break;
                        }
                }
                else
                        return 0;
        }

        //应用程序阶段：负责处理需要绘制的图元及模型、相机和光源位置
        Eigen::Vector3f eye_pos = {0, 0, 5};

        //三角形点位置
        std::vector<Eigen::Vector3f> pos{
            {2, 0, -2}, {0, 2, -2}, {-2, 0, -2}, {3.5, -1, -5}, {2.5, 1.5, -5}, {-1, 0.5, -5}};

        //索引
        std::vector<Eigen::Vector3i> ind{
            {0, 1, 2},
            {3, 4, 5}};

        //颜色值
        std::vector<Eigen::Vector3f> cols{
            {217.0, 238.0, 185.0},
            {217.0, 238.0, 185.0},
            {217.0, 238.0, 185.0},
            {185.0, 217.0, 238.0},
            {185.0, 217.0, 238.0},
            {185.0, 217.0, 238.0}};

        rst::rasterizer r(700, 700);

        auto pos_id = r.load_positions(pos);
        auto ind_id = r.load_indices(ind);
        auto col_id = r.load_colors(cols);
        r.set_msaa(msaa);

        int key = 0;
        int frame_count = 0;

        if (command_line)
        {
                r.clear(rst::Buffers::Color | rst::Buffers::Depth);

                //模型变换
                r.set_model(get_model_matrix(angle));
                //视图变换
                r.set_view(get_view_matrix(eye_pos));
                //顶点着色
                //投影变换
                r.set_projection(get_projection_matrix(45, 1, 0.1, 50, isoth));
                //裁剪
                //屏幕映射

                r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
                cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
                cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

                cv::imwrite(filename, image);

                return 0;
        }

        while (key != 27)
        {
                r.clear(rst::Buffers::Color | rst::Buffers::Depth);

                //模型变换
                r.set_model(get_model_matrix(angle));
                //视图变换
                r.set_view(get_view_matrix(eye_pos));
                //顶点着色
                //投影变换
                r.set_projection(get_projection_matrix(45, 1, 0.1, 50, isoth));
                //裁剪
                //屏幕映射

                r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
                cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
                cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

                cv::imshow("image", image);
                key = cv::waitKey(10);

                std::cout << "frame count: " << frame_count++ << '\n';
        }

        return 0;
}
