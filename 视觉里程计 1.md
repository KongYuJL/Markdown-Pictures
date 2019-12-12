## 视觉里程计 1

第七章主要讲的是SLAM中的前端：**视觉里程计（visual odometry）**, 简称VO。
VO非常重要，与同时定位与建图中的其一，定位，密切相关。VO的输入可以是单张RGB图片，也可以是RGBD，还可以包含IMU数据，甚至可以是前后两张图片同时输入（也许也能是视频流）。

所谓IMU，就是惯性测量单元，比如陀螺仪等等，计算定位对象的加速度之类的信息。最后输出的是定位对象的位姿信息，就是旋转向量和平移向量。

不过VO预测的是相对运动，在绝对定位场景中，会设第一帧图像的位置为参考坐标系，或者以第一帧图像的相对位置作为参考坐标系。

如何将图片之类的输入数据，处理成我们想要的输出，位姿信息？这里面就有很多实现方法，主要分为两种，一种是特征点法，一种是不提取特征的直接法。目前的主流是基于特征点法，更有效、鲁棒。

本章主要讲了以下内容：

1. 特征点法：找到两张2D图像上的匹配点。
2. 对极几何：根据2D-2D特征点对求解R,t。
3. 三角测量：根据2D-2D特征点求深度。
4. PnP：根据3D点云和匹配的2D图像求R,t。
5. ICP：求两个点云之间的R,t。
其关系可参考下图，来自[博客](https://blog.csdn.net/qq_23225073/article/details/78452638#31-%E6%9C%AC%E8%B4%A8%E7%9F%A9%E9%98%B5essential-matrixe)：
![参考图片](https://img-blog.csdn.net/20171113122733673?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvcXFfMjMyMjUwNzM=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)

### 1.特征点法

什么是特征点呢？首先，计算机中的图像，本质是一个由亮度和色彩组成的多维矩阵。如果直接从矩阵层次考虑运动估计，效率会很低，会引入许多噪声。因此传统的解决方案，是先从图像中选取比较**有代表性**的点。这些点通常具有**可重复性、可区别性、高效率、本地性**。在经典SLAM中，也称为**路标**，在视觉SLAM中，则指**图像特征**(Feature)。

特征点通常由关键点（key-point）和描述子(descriptor)两部分组成。先提取“关键点”，再计算“描述子”。常见的特征点有**SIFT**(尺度不变特征变换, Scale-Invariant Feature Transform)，**SURF**(Speeded up Robust Features), **ORB**(Oriented Fast and Rotated BRIEF)，其中ORB是最具有代表性的实时图像特征（SIFT太慢，FAST没有方向信息）。

#### 1.1 ORB特征

提取ORB特征的两个步骤：

+ **Oriented FAST角点（关键点）提取**

+ **BRIEF描述子**

**关键点：Oriented FAST**

ORB特征使用的FAST关键点是改进的Oriented FAST。FAST主要检测局部像素灰度变化明显的地方，以速度快著称。它的核心思想是：如果一个像素与邻域像素的差别较大（过亮或过暗），那么它更可能是角点（关键点）。具体的流程见书。

![FAST特征点](https://img-blog.csdn.net/20170220092211672?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvYzYwMjI3MzA5MQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)

原始的FAST角点经常出现“扎堆”的现象，还需要使用NMS(极大值抑制，non-maximal suppresion)，避免角点集中问题。ORB还对原始FAST角点计算**Harris响应值**，然后选取前N个具有最大响应值的角点。此外，改进的FAST关键点添加了方向和尺度信息。尺度不变性由构建图像金字塔（多次不同层次的降采样），并在金字塔的每一层上检测关键点来实现。而特征的旋转由灰度质心法来表示，步骤如下：

(1) 在一个小的图像块B中，定义图像块的矩

$$
m_{p q}=\sum x^{p} y^{q} I(x, y), \ \ \ p, q= \{0,1\}
$$

(2) 计算图像块的质心
$$
C=\left(\frac{m_{10}}{m_{00}}, \frac{m_{01}}{m_{00}}\right)=\left(\frac{\sum x I(x, y)}{\sum I(x, y)}, \frac{\sum y I(x, y)}{\sum I(x, y)}\right)
$$
(3) 计算特征点方向，连接图像块的几何中心O和质心C，可以得到方向向量 $\vec{OC}$，因此特征点的方向可以定义为：
$$
\theta=\arctan \left(\frac{m_{01}}{m_{10}}\right)=\arctan \left(\frac{\sum y I(x, y)}{\sum x I(x, y)}\right)
$$
这样得到的Oriented FAST关键点，具有更好的鲁棒性。

**描述子：BRIEF**

**BRIEF**(Binary Robust Independent Elementary Feature)，是一种二进制描述子，描述向量是由许多个0和1组成，这里的0和1编码了**关键点附近的两个像素**$p$和$q$的大小关系，如果$p>q$,则取1；反之取0。具体像素的挑选是按照某个特定的分布进行随机，具体可以查看OpenCV代码或者原论文。

#### 1.2 特征匹配

特征点是具有代表性的点（相当于人现实世界定位时使用的参照物），而特征匹配解决了SLAM中的数据关联问题(data association)，即确定当前帧的图像特征与前一帧的图像对应关系，通过描述子的差异判断哪些特征为同一个点。再使用相同特征点的相对位置关系，就可以计算出RT（对极几何）。

在图像$I_t$中提取到特征点$x_t^m,m=1,2,...M$，在图像$I_{t+1}$中提取到特征点$x_{t+1}^n,n=1,2,...N$，寻找这两个集合之间的对应关系就是特征匹配。最简单的方式是使用暴力匹配方法(Brute-Force Matcher)。即对两个集合之间的所有点计算描述子的距离，然后排序，取最近的点。**描述子距离**表示两个特征之间的相似程度，在实际应用中取不同的距离度量范数。

+ 浮点类型的描述子使用欧式距离度量
+ 二进制类型描述子(eg: BRIEF)使用汉明距离(Hamming Distance)作为度量

特征点数量过多时，暴力匹配算法会非常慢，不满足SLAM的实时性要求，因此OpenCV中使用的是**快速最近邻方法（FLANN）**。但由于图像特征的局部特性，误匹配的情况广泛存在，比如场景中经常存在大量的重复纹理，使得特征非常相似。因此，这种情况下仅利用局部特征解决误匹配是非常困难的。

### 2. 2D-2D：对极几何

#### 2.1 对极约束

