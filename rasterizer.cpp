#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
        auto id = get_next_id();
        pos_buf.emplace(id, positions);

        return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
        auto id = get_next_id();
        ind_buf.emplace(id, indices);

        return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
        auto id = get_next_id();
        col_buf.emplace(id, cols);

        return {id};
}

//转换为齐次矩阵
auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f)
{
        return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//计算重心坐标
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f *v)
{
        float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
        float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
        float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
        return {c1, c2, c3};
}

//判断点是否在三角形内
static bool insideTriangle(int x, int y, const Vector3f *_v)
{
        Eigen::Vector2f p;
        p << x, y;
        Eigen::Vector2f ab, bc, ca;
        ab = _v[1].head(2) - _v[0].head(2);
        bc = _v[2].head(2) - _v[1].head(2);
        ca = _v[0].head(2) - _v[2].head(2);

        Eigen::Vector2f ap, bp, cp;
        ap = p - _v[0].head(2);
        bp = p - _v[1].head(2);
        cp = p - _v[2].head(2);

        auto aa = ab[0] * ap[1] - ab[1] * ap[0];
        auto bb = bc[0] * bp[1] - bc[1] * bp[0];
        auto cc = ca[0] * cp[1] - ca[1] * cp[0];

        //return (aa > 0) && (bb > 0) && (cc > 0);
        return (aa * bb > 0) && (aa * cc > 0) && (bb * cc > 0);
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
        auto x1 = begin.x();
        auto y1 = begin.y();
        auto x2 = end.x();
        auto y2 = end.y();

        Eigen::Vector3f line_color = {255, 255, 255};

        int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

        dx = x2 - x1;
        dy = y2 - y1;
        dx1 = fabs(dx);
        dy1 = fabs(dy);
        px = 2 * dy1 - dx1;
        py = 2 * dx1 - dy1;

        if (dy1 <= dx1)
        {
                if (dx >= 0)
                {
                        x = x1;
                        y = y1;
                        xe = x2;
                }
                else
                {
                        x = x2;
                        y = y2;
                        xe = x1;
                }
                Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
                set_pixel(point, line_color);
                for (i = 0; x < xe; i++)
                {
                        x = x + 1;
                        if (px < 0)
                        {
                                px = px + 2 * dy1;
                        }
                        else
                        {
                                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                                {
                                        y = y + 1;
                                }
                                else
                                {
                                        y = y - 1;
                                }
                                px = px + 2 * (dy1 - dx1);
                        }
                        //            delay(0);
                        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
                        set_pixel(point, line_color);
                }
        }
        else
        {
                if (dy >= 0)
                {
                        x = x1;
                        y = y1;
                        ye = y2;
                }
                else
                {
                        x = x2;
                        y = y2;
                        ye = y1;
                }
                Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
                set_pixel(point, line_color);
                for (i = 0; y < ye; i++)
                {
                        y = y + 1;
                        if (py <= 0)
                        {
                                py = py + 2 * dx1;
                        }
                        else
                        {
                                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                                {
                                        x = x + 1;
                                }
                                else
                                {
                                        x = x - 1;
                                }
                                py = py + 2 * (dx1 - dy1);
                        }
                        //            delay(0);
                        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
                        set_pixel(point, line_color);
                }
        }
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::col_buf_id col_buffer, rst::Primitive type)
{
        if (type != rst::Primitive::Triangle)
        {
                throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
        }
        auto &buf = pos_buf[pos_buffer.pos_id];
        auto &ind = ind_buf[ind_buffer.ind_id];
        auto &col = col_buf[col_buffer.col_id];

        float f1 = (50 - 0.1) / 2.0;
        float f2 = (50 + 0.1) / 2.0;

        Eigen::Matrix4f mvp = projection * view * model;
        for (auto &i : ind)
        {
                Triangle t;

                Eigen::Vector4f v[] = {
                    mvp * to_vec4(buf[i[0]], 1.0f),
                    mvp * to_vec4(buf[i[1]], 1.0f),
                    mvp * to_vec4(buf[i[2]], 1.0f)};

                //Homogeneous division
                for (auto &vec : v)
                {
                        vec /= vec.w();
                }

                //Viewport transformation
                for (auto &vert : v)
                {
                        vert.x() = 0.5 * width * (vert.x() + 1.0);
                        vert.y() = 0.5 * height * (vert.y() + 1.0);
                        vert.z() = vert.z() * f1 + f2;
                }

                for (int i = 0; i < 3; ++i)
                {
                        t.setVertex(i, v[i].head<3>());
                        t.setVertex(i, v[i].head<3>());
                        t.setVertex(i, v[i].head<3>());
                }

                auto col_x = col[i[0]];
                auto col_y = col[i[1]];
                auto col_z = col[i[2]];

                t.setColor(0, col_x[0], col_x[1], col_x[2]);
                t.setColor(1, col_y[0], col_y[1], col_y[2]);
                t.setColor(2, col_z[0], col_z[1], col_z[2]);

                rasterize_wireframe(t, get_msaa());
        }
}

void rst::rasterizer::rasterize_wireframe(const Triangle &t, const MSAA &msaa)
{
        auto v = t.toVector4();
        // TODO : Find out the bounding box of current triangle.
        // iterate through the pixel and find if the current pixel is inside the triangle
        auto minx = std::min(v[0][0], std::min(v[1][0], v[2][0]));
        auto miny = std::min(v[0][1], std::min(v[1][1], v[2][1]));
        auto maxx = std::max(v[0][0], std::max(v[1][0], v[2][0]));
        auto maxy = std::max(v[0][1], std::max(v[1][1], v[2][1]));
        int startx = (int)std::floor(minx);
        int endx = (int)std::ceil(maxx);
        int starty = (int)std::floor(miny);
        int endy = (int)std::ceil(maxy);
        //MSAA2×，4×，8×，16×
        std::vector<Eigen::Vector2f> pos{{0.5, 0.5}};
        int samples = 1;
        switch (msaa)
        {
        case 2:
                /* code */
                pos = std::vector<Eigen::Vector2f>{
                    {0.25, 0.25}, {0.75, 0.75}};
                samples = 2;
                break;
        case 4:
                pos = std::vector<Eigen::Vector2f>{
                    {0.25, 0.25}, {0.25, 0.75}, {0.75, 0.25}, {0.75, 0.75}};
                samples = 4;
                break;
        case 8:
                pos = std::vector<Eigen::Vector2f>{
                    {0.25, 0.25}, {0.25, 0.5}, {0.25, 0.75}, {0.5, 0.25}, {0.5, 0.75}, {0.75, 0.25}, {0.75, 0.5}, {0.75, 0.75}};
                samples = 8;
                break;
        default:
                pos = std::vector<Eigen::Vector2f>{
                    {0.5, 0.5}};
                samples = 1;
                std::cout << "No MSAA!" << std::endl;
                break;
        }
        for (int i = startx; i <= endx; i++)
        {
                for (int j = starty; j <= endy; j++)
                {
                        int count = 0;
                        float mindpth = FLT_MAX;
                        for (int ind = 0; ind < samples; ind++)
                        {
                                if (insideTriangle((float)i + pos[ind][0], (float)j + pos[ind][1], t.v))
                                {
                                        // If so, use the following code to get the interpolated z value.
                                        auto barycentric = computeBarycentric2D((float)i + pos[ind][0], (float)j + pos[ind][1], t.v);
                                        auto alpha = std::get<0>(barycentric);
                                        auto beta = std::get<1>(barycentric);
                                        auto gamma = std::get<2>(barycentric);
                                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                                        z_interpolated *= w_reciprocal;
                                        mindpth = std::min(mindpth, z_interpolated);
                                        count++;
                                }
                        }
                        if (count != 0)
                        {
                                int index = get_index(i, j);
                                if (depth_buf[index] > mindpth)
                                {
                                        Vector3f color = t.getColor() * count / samples;
                                        Vector3f point(3);
                                        point << (float)i, (float)j, mindpth;
                                        depth_buf[index] = mindpth;
                                        set_pixel(point, color);
                                }
                        }

                        // if (insideTriangle((float)i + 0.5, (float)j + 0.5, t.v))
                        // {
                        //         // If so, use the following code to get the interpolated z value.
                        //         auto barycentric = computeBarycentric2D((float)i + 0.5, (float)j + 0.5, t.v);
                        //         auto alpha = std::get<0>(barycentric);
                        //         auto beta = std::get<1>(barycentric);
                        //         auto gamma = std::get<2>(barycentric);
                        //         float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        //         float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        //         z_interpolated *= w_reciprocal;
                        //         // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                        //         int index = get_index(i, j);
                        //         if (depth_buf[index] > z_interpolated)
                        //         {
                        //                 Vector3f color = t.getColor();
                        //                 Vector3f point(3);
                        //                 point << (float)i, (float)j, z_interpolated;
                        //                 depth_buf[index] = z_interpolated;
                        //                 set_pixel(point, color);
                        //         }
                        // }
                }
        }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m)
{
        model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v)
{
        view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p)
{
        projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
        if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
        {
                std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        }
        if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
        {
                std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
        frame_buf.resize(w * h);
        depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
        return (height - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color)
{
        //old index: auto ind = point.y() + point.x() * width;
        if (point.x() < 0 || point.x() >= width ||
            point.y() < 0 || point.y() >= height)
                return;
        auto ind = (height - point.y()) * width + point.x();
        frame_buf[ind] = color;
}
