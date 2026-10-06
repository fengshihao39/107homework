# 107homework

## ROS 2 与 FAST-LIO 知识图

以下三张图整理了 Livox 驱动、Topic、FAST-LIO 和本项目分析节点之间的关系。

### 1. 数据流与 Topic：谁把数据发给谁？

![MID360、Livox 驱动、FAST-LIO 与分析节点的数据流及 Topic 关系](docs/images/01-data-flow.png)

### 2. FAST-LIO：运动预测与几何残差纠正

![FAST-LIO 的 IMU 积分预测、点云运动补偿、地图匹配与迭代卡尔曼滤波更新](docs/images/02-fastlio-estimation.png)

### 3. 分析节点：加速度大小与位置差分速度

![imu_analysis 和 odom_analysis 的输入、计算方法与输出话题](docs/images/03-analysis-nodes.png)

在本项目的数据处理流程中，迭代卡尔曼滤波在 FAST-LIO 内部进行；`imu_analysis` 和 `odom_analysis` 提取并发布分析指标，不参与 FAST-LIO 的滤波和定位。
