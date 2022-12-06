# 功能
1. 测试雷达点云的数据，打印雷达点云的各种属性
2. 从SLAM算法中提取reconstruction所需的相机位姿和图片
3. 可视化手动调整外参，在终端中`Ctrl+C`输出调整之后的外参结果(旋转矩阵、欧拉角、位移向量)
  
# 操作步骤
1. 创建工作空间
```bash
mkdir catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```
2. 克隆代码并切换到reconstruct分支
```bash
git clone https://github.com/KeepOnKeepOn/test_tools.git
git checkout origin/reconstruct
git checkout -b reconstruct
```
3. 编译
```bash
cd ..
catkin_make
```
4. 运行launch文件
```bash
roslaunch test_tools run.launch
```
注意修改launch文件中的参数以适配
   - map_frame_id: 世界坐标系名称
   - IMU_frame_id: IMU坐标系名称
   - camera_frame_id: 相机坐标系名称 
   - pointcloud_topic: 雷达点云话题
   - image_topic： 原始图像话题
   - compressed_topic: 压缩格式图像话题，与原始图像话题只需其一即可
   - static_transform_publisher节点发布相机坐标系到IMU坐标系的坐标转换