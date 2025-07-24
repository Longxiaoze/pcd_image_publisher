# pcd_image_publisher

这个储存库用来发布一对指定的pcd和image来测试标定

## 安装
``` bash
mkdir -p ~/color_pointcloud_360_ws/src
cd ~/color_pointcloud_360_ws/src
git clone https://github.com/Longxiaoze/pcd_image_publisher.git
cd ~/color_pointcloud_360_ws
colcon build
source install/setup.bash
```

## 运行
``` bash
ros2 run pcd_image_publisher pcd_image_publisher   --ros-args     -p pcd_path:=/home/longxiaoze/color_lidar_ws/src/demo_datas/save_pcd/cloud_0000.pcd     -p image_path:=/home/longxiaoze/color_lidar_ws/src/demo_datas/20250723_145351_330.jpg     -p pcd_frame:=lidar     -p image_frame:=camera \
```

发布到：

/camera/image_raw

/points_raw

## vis
change `Fixed Frame` to `lidar` in rviz2

and you can see no color pcd in rviz2
(If you can not see pcd, you need to click twice for images)