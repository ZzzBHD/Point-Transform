# Point-Transform
Conversion between different Point Cloud types
## 上传并备份一些不同类型点云之间转换的实现代码<br>
针对以下点云类型的转换，实现均在launch文件夹中，在转换前记得修改file/pointcloud type、topic、number、filepath等参数<br>
### Tran_pointcloud_c<br>
1：bin2pcd/pcd2bin <br>
2：tof2pcd、bin<br>
3：pub(bag)2pcd、bin<br>
### Tran_pointcloud_c2p<br>
1：pcd2npy<br>
注：bin2npy会由于bin的读取速度快，导致在pub的时候python节点还没有初始化完成，导致没有转换，未来可能的话加一个feedback实现<br>
### Tran_pointcloud_p2c<br>
1: npy2pcd、bin<br>
### lidar_to_pcd<br>
对激光雷达-->pcd的代码进行备份，实际可以用Tran_pointcloud_c实现
