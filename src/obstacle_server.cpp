#include <ros/ros.h>
#include <multipoint_navigation_receiver/obstacle_srv.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/thread/thread.hpp>
nav_msgs::OccupancyGrid mapdata;

bool obstacleCallback(multipoint_navigation_receiver::obstacle_srv::Request &req,
                      multipoint_navigation_receiver::obstacle_srv::Response &res)
{
    if (mapdata.data.size() - 0 < 0.001)
    {
        return false;
    }
    
    int width = mapdata.info.width;
    std::vector<signed char> arr = mapdata.data;
    float x = req.positionx;
    float y = req.positiony;
    printf("positionx is %f and positiony is %f \n",x,y);
    int pos = (int)((req.positiony - mapdata.info.origin.position.y) * width * (1.00/mapdata.info.resolution) + (req.positionx - mapdata.info.origin.position.x) * (1.00/mapdata.info.resolution));
    int obstacle = (int)arr.at(pos);
    printf("pos is %d and obstacle is %d\n",pos,obstacle);
    res.obstacle_probability = obstacle;
    
    return true;
}

void sub_once()
{
    boost::shared_ptr<nav_msgs::OccupancyGrid const> batEdge;    // 有const
    batEdge = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap",ros::Duration(3));
    if(batEdge != NULL)
    {
        mapdata = *batEdge;
        printf("update mapdate \n");
    }
    else
        printf("no topic\n");
}
void subThread()
{
    while(ros::ok())
    {
        sub_once();
        sleep(30);
    }
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "obstacle_server");
    ros::NodeHandle n;
    boost::thread thrd(&subThread);

    ros::ServiceServer obstacle_server = n.advertiseService("/obstacle_information",obstacleCallback);
    ros::spin();
    return 0;
}



