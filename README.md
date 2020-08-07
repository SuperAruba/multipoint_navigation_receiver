# multipoint_navigation_receiver
基于ROS的多点导航功能包  

## 下载和编译
```
cd catkin_ws/src
git clone https://github.com/SuperAruba/multipoint_navigation_receiver.git
cd ..
catkin_make
```  

## 使用说明
  多点导航需要用到multipoint_navigation_receiver节点，由于考虑到目标点可能出现长时间存在障碍物的情况，故增加了一个判断地图中某点德障碍物信息德节点obstacle_server节点。
  obstacle_server节点订阅了/move_base/global_costmap/costmap话题，启动服务机制，服务名称为/obstacle_information，请求服务时参数为地图某点的X坐标和Y坐标。该节点可以直接用于其他节点请求地图中德障碍物数据。该几点依赖于global_costmap，所以需要在global_costmap中加入obstacle layer。
  multipoint_navigation_receiver节点使用时需要先开启obstacle_server节点，传入的参数为坐标和姿态一一对应的两个数组（考虑到某些其他语言无法直接得到move_base_msgs::MoveBaseGoal数据类型）分别为geometry_msgs/Vector3[]和geometry_msgs/Quaternion[]。
  如需停止当前多点导航，发送两个空的数组即可。
