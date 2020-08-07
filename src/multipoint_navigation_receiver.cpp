#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <queue>
#include <multipoint_navigation_receiver/obstacle_srv.h>
#include <multipoint_navigation_receiver/set_goal_srv.h>
#include <signal.h>
#include <pthread.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

using namespace std;
Client* client;
ros::NodeHandle* n;
void* send_goals(void* goals);
pthread_t send_thread;
bool thread_isexist;
void MySigintHandler(int sig)
{
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
	ROS_INFO("shutting down!");
	ros::shutdown();
}

bool goalsCB(multipoint_navigation_receiver::set_goal_srv::Request &req,multipoint_navigation_receiver::set_goal_srv::Response &res)
{
  if (req.positions.size() == 0)
  {
    if (thread_isexist)
    {
      pthread_cancel(send_thread);
      while (pthread_kill(send_thread,0) == 0)
      {
        sleep(0.2);
      }
    }
    client->cancelGoal();
    thread_isexist = false;
    res.result = "cancel received";
    return true;
  }
  queue<move_base_msgs::MoveBaseGoal>* goals = new queue<move_base_msgs::MoveBaseGoal>();
  for (size_t i = 0; i < req.positions.size(); i++)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = req.positions[i].x;
    goal.target_pose.pose.position.y = req.positions[i].y;
    goal.target_pose.pose.position.z = req.positions[i].z;
    goal.target_pose.pose.orientation.x = req.quaternions[i].x;
    goal.target_pose.pose.orientation.y = req.quaternions[i].y;
    goal.target_pose.pose.orientation.z = req.quaternions[i].z;
    goal.target_pose.pose.orientation.w = req.quaternions[i].w;
    ROS_INFO("mapdata:px=%lf,py=%lf,pz=%lf,qx=%lf,qy=%lf,pz=%lf,pw=%lf",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,goal.target_pose.pose.position.z,
    goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y,goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
    goals->push(goal);
  }
   res.result = "goals received";
   if (!thread_isexist)
   {
     pthread_create(&send_thread,NULL,send_goals,goals);
   }else
   {
     pthread_cancel(send_thread);
     while (pthread_kill(send_thread,0) == 0)
     {
       sleep(2);
     }
     pthread_create(&send_thread,NULL,send_goals,goals);
   }
   return true;
}

int main(int argc, char** argv)
{
  thread_isexist = false;
  ros::init(argc, argv, "mymove_base_client");
  n = new ros::NodeHandle();
  ROS_INFO("start");
  ros::service::waitForService("/obstacle_information");
  ros::ServiceServer goals_server = n->advertiseService("/goals_server",goalsCB);
  client = new Client("move_base", true); // true -> don't need ros::spin()
  client->waitForServer(ros::Duration(60.0));
  signal(SIGINT, MySigintHandler);
  ros::spin();
  return 0;
}
void* send_goals(void* data)
{
    thread_isexist = true;
    queue<move_base_msgs::MoveBaseGoal>* goals;
    goals =(queue<move_base_msgs::MoveBaseGoal>*) data;
    ros::ServiceClient obstacle_client = n->serviceClient<multipoint_navigation_receiver::obstacle_srv>("/obstacle_information");
    client->sendGoal(goals->front());
    int count = 0;
    while (ros::ok())
    {
      pthread_testcancel();
      bool Arrivals = client->waitForResult(ros::Duration(60.0));
      if (!Arrivals)
      {
        client->cancelGoal();
        multipoint_navigation_receiver::obstacle_srv obstacle;
        obstacle.request.positionx = goals->front().target_pose.pose.position.x;
        obstacle.request.positiony = goals->front().target_pose.pose.position.y;
        obstacle_client.call(obstacle);
        int8_t obstacle_probability = obstacle.response.obstacle_probability;
        printf("obstacle is %d \n",obstacle_probability);
        if(obstacle_probability > 0)
        {
          printf("Goal have obatacle! \n");

          goals->pop();
          if (goals->empty())
          {
            printf("You got all goals \n");
            delete(goals);
            break;
          }else
          {
            client->sendGoal(goals->front());
            count = 0;
          }
          printf("Timed out achieving goal, go next \n");
        }
        else if(obstacle_probability == 0)
        {
          client->sendGoal(goals->front());
          count = 0;
          printf("You resent one goal\n");
        }
      }
      else
      {
        count ++ ;
        if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          printf("Yay! One goal OK!\n");
          goals->pop();
          if (goals->empty())
          {
            printf("You got all goals\n");
            delete(goals);
            break;
          }
          else
          {
            client->sendGoal(goals->front());
            count = 0;
          }
        }
        else if(count > 3)
        {       
          client->sendGoal(goals->front());
          count = 0;
          printf("You resent one goal\n");
        }
        printf("Current State: %s\n", client->getState().toString().c_str());
      }
    }
  thread_isexist = false;
  return 0;
}
