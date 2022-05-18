# SC-A-LOAM：在A-LOAM中加入回环检测

发布于2022-02-10 12:26:18阅读 1450



Thanks to LOAM, A-LOAM, and LIO-SAM code authors. The major codes in this repository are borrowed from their efforts.

**代码：**https://github.com/gisbi-kim/SC-A-LOAM

编译：点云PCL

本文仅做学术分享，如有侵权，请联系删除。

**本文来自点云PCL博主的分享，未经作者允许请勿转载，欢迎各位同学积极分享和交流。**

摘要

该开源库是在A-LOAM的基础上在增加了回环检测和位姿图优化模块（名为SC-PGO）

该库也在FAST-LIO2激光里程计中集成了。其代码位于 ：

https://github.com/gisbi-kim/SC-A-LOAM

https://github.com/gisbi-kim/FAST_LIO_SLAM

主要内容及贡献

实时激光雷达SLAM集成了A-LOAM和ScanContext。

- A-LOAM用于里程计模块（即连续运动估计）
- ScanContext用于处理大漂移的粗略全局定位（即无初始姿势的机器人位置识别问题）
- 并将GTSAM的iSAM2用于位姿图优化。

此库旨在展示ScanContext的便捷适用性。

- 用户应该做的唯一事情就是包括Scancontext.h调用makeAndSaveScancontextAndKeys 和 detectLoopClosureID。

**SC-A-LOAM特点**

- 鲁棒的位置识别和回环闭合：将ScanContext作为回环检测器集成到A-LOAM中，然后进行基于ISAM2的姿势图优化。
- 模块化实现：与A-LOAM的唯一区别是添加了laserPosegraphOptimization.cpp文件,在新文件中，订阅了点云topic和里程计topic（订阅了从laserMapping.cpp发布的A-LOAM的结果）。也就是说，实现了对于任何前端里程计方法都是通用的，因此，我们的姿势图优化模块（即laserPosegraphOptimization.cpp）可以轻松地与任何里程计算法集成，如甚至非LOAM系列或甚至其他传感器（例如视觉里程计）。

![img](https://ask.qcloudimg.com/http-save/yehe-5926470/49b96d0f907f1abcf5d5bfcaec841691.png?imageView2/2/w/1620)

- 使用消费级GPS进行高度值稳定：为了使结果更加可信，模块支持基于GPS（消费者级价格，如U-Blox EVK-7P）的高度值稳定，众所周知，LOAM系列方法在室外易受z轴值误差的影响，这里仅对高度值使用稳健损失，有关详细信息，可参考laserPosegraphOptimization.cpp文件。

**依赖**

主要依赖ROS、Ceres（用于A-LOAM）和GTSAM（用于姿势图优化）。

实验

**MulRan数据集**

提供了激光雷达扫描点云（ Ouster OS1-64，水平安装，10Hz）和消费者级gps（U-Blox EVK-7P，4Hz）数据。

![img](https://ask.qcloudimg.com/http-save/yehe-5926470/ac0a9c07dde527b8ffb949773374e4ba.png?imageView2/2/w/1620)

 **KITTI (HDL-64 获取点云数据)**

![img](https://ask.qcloudimg.com/http-save/yehe-5926470/cdb500d20d12f893fe330d82cec35989.png?imageView2/2/w/1620)

**室内场景**

![img](https://ask.qcloudimg.com/http-save/yehe-5926470/e16b698a75eb60a6ed447c86267cb8f2.png?imageView2/2/w/1620)

**数据保存和地图构建**

支持每个关键帧的位姿和扫描点云数据的保存，使用这些保存的数据，可以离线构建地图（在ROI内）。请参阅utils/python/makeMergedMap.py和对应教程。下面是MulRan数据集KAIST 03的合并地图的示例结果，使用CloudCompare可视化结果。

![img](https://ask.qcloudimg.com/http-save/yehe-5926470/ae8a8fd9f3623a8c6a89883714ad7d51.png?imageView2/2/w/1620)

总结

A-LOAM的基础上在增加了回环检测和位姿图优化模块。也是LOAM系列SLAM方案的扩展和优化。