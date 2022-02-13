# 101Assignment5

Games101作业1-5，主要实现绘制三角形和MSAA反走样。

已实现内容：
* 三角形绘制
* MSAA(2x,4x,8x)

# MSAA8x渲染图
![MSAA8x](https://github.com/Auggst/101Assignment5/blob/master/output/8xMSAA.png)

# 环境
wsl2子系统或linux系统
```
git clone https://github.com/Auggst/101Assignment5.git
mkdir build && cd build
camke ..
make -j4
./GraphicsHome  -r 30 output.png -p  false -MSAA 2
```
-r 表示三角形旋转角度
-p 是否采用透视投影
-MSAA 采用MSAA反走样(分为2，4，8及其他，其他为不采用MSAA)

* opencv 
* Eigen库
* cmake 3.10及以上
