#pragma once

#include <algorithm>
#include <Eigen/Eigen>
using namespace Eigen;

namespace cam
{
        class camera
        {
        public:
                camera(Eigen::Vector3f lookfrom,
                       Eigen::Vector3f lookat,
                       Eigen::Vector3f vup)
                {
                        origin = lookfrom;

                        w = (lookfrom - lookat).normalized();
                        u = vup.cross(w);
                        v = w.cross(u);
                };

        private:
                Eigen::Vector3f origin;         //相机位置
                Eigen::Vector3f u, v, w;        //相机坐标系三分量
                
        };
} // namespace cam
