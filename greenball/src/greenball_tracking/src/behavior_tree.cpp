#include <iostream>
#include <unistd.h>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/leaf_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include <ros/ros.h>
#include <chrono>
#include "geometry_msgs/Point.h"

#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <PahoMQTT.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

using namespace UNITREE_LEGGED_SDK;


//Global variables
geometry_msgs::Point last_msg_;
ros::Publisher pub_high_;
ros::Time last_message_time;
bool _haltRequested = false;

// Callback function for the ball coordinates topic
void ballPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
  last_message_time = ros::Time::now();
  last_msg_ = *msg;
}


//Ball found condition node

BT::NodeStatus BallFound(BT::TreeNode &node) {
  ros::spinOnce();

  ros::Time current_time = ros::Time::now();

  // Check if the ball is close
  if ((current_time - last_message_time).toSec() < 0.5) {
    ros::spinOnce();
    std::cout << "Ball Found" << std::endl;
/*
    unitree_legged_msgs::HighCmd high_cmd_stop;

    for (size_t i = 0; i < 3; i++)
    {
      high_cmd_stop.head[0] = 0xFE;
      high_cmd_stop.head[1] = 0xEF;
      high_cmd_stop.levelFlag = HIGHLEVEL;
      high_cmd_stop.mode = 0;
      high_cmd_stop.gaitType = 0;
      high_cmd_stop.speedLevel = 0;
      high_cmd_stop.footRaiseHeight = 0;
      high_cmd_stop.bodyHeight = 0;
      high_cmd_stop.euler[0] = 0.0;
      high_cmd_stop.euler[1] = 0.0;
      high_cmd_stop.euler[2] = 0.0;
      high_cmd_stop.velocity[0] = 0.0f;
      high_cmd_stop.velocity[1] = 0.0f;
      high_cmd_stop.yawSpeed = 0.0f;
      high_cmd_stop.reserve = 0;

      pub_high_.publish(high_cmd_stop);
    }
*/    
    return BT::NodeStatus::SUCCESS;

  } else {
    ros::spinOnce();
    std::cout << "Ball Not Found" << std::endl;
    std::cout << "Running: Searching for ball..." << std::endl;
    
    return BT::NodeStatus::FAILURE;
  }
}


//Find ball action node

class SearchBall : public BT::AsyncActionNode
{
public:
  explicit SearchBall(const std::string &name) : BT::AsyncActionNode(name, {})
  {
  }

  void halt() override
  {
    _haltRequested = true;
  }

  BT::NodeStatus tick() override
  {
    if (_haltRequested)
    {
      return BT::NodeStatus::IDLE;
    }

    auto start = std::chrono::high_resolution_clock::now();

    while ((ros::ok()) && (!_haltRequested))
    {
      ros::spinOnce();
      ros::Time current_time = ros::Time::now();

      if ((current_time - last_message_time).toSec() < 0.5)
      {
        return BT::NodeStatus::SUCCESS;
      }

      unitree_legged_msgs::HighCmd high_cmd_stop;
      
      for (size_t i = 0; i < 3; i++)
      {
        high_cmd_stop.head[0] = 0xFE;
        high_cmd_stop.head[1] = 0xEF;
        high_cmd_stop.levelFlag = HIGHLEVEL;
        high_cmd_stop.mode = 2;
        high_cmd_stop.gaitType = 1;
        high_cmd_stop.speedLevel = 0;
        high_cmd_stop.footRaiseHeight = 0;
        high_cmd_stop.bodyHeight = 0;
        high_cmd_stop.euler[0] = 0.0;
        high_cmd_stop.euler[1] = 0.0;
        high_cmd_stop.euler[2] = 0.0;
        high_cmd_stop.velocity[0] = 0.0f;
        high_cmd_stop.velocity[1] = 0.0f;
        high_cmd_stop.yawSpeed = 0.5f;
        high_cmd_stop.reserve = 0;

        pub_high_.publish(high_cmd_stop);
      
      }
      
      // check if the timeout has been exceeded
      auto now = std::chrono::high_resolution_clock::now();
      float _timeout = 5.0; 
      if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() >= _timeout)
      {
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

private:
  bool _haltRequested;
};


//Ball close condition node

BT::NodeStatus ballClose(BT::TreeNode &node) {
  ros::spinOnce();

  ros::Time current_time = ros::Time::now();

  // Check if the ball is close
  if (last_msg_.z > 30) {
    ros::spinOnce();
    std::cout << "Ball Close" << std::endl;

    unitree_legged_msgs::HighCmd high_cmd_stop;

    for (size_t i = 0; i < 3; i++)
    {
      if (last_msg_.y > 270)
      {
        cout<<"Look Down "<<endl;
        high_cmd_stop.head[0] = 0xFE;
        high_cmd_stop.head[1] = 0xEF;
        high_cmd_stop.levelFlag = HIGHLEVEL;
        high_cmd_stop.mode = 1;
        high_cmd_stop.gaitType = 0;
        high_cmd_stop.speedLevel = 0;
        high_cmd_stop.footRaiseHeight = 0;
        high_cmd_stop.bodyHeight = 0;
        high_cmd_stop.euler[0] = 0.0;
        high_cmd_stop.euler[1] = 0.3;
        high_cmd_stop.euler[2] = 0.0;
        high_cmd_stop.velocity[0] = 0.0f;
        high_cmd_stop.velocity[1] = 0.0f;
        high_cmd_stop.yawSpeed = 0.0f;
        high_cmd_stop.reserve = 0;

      } else if (last_msg_.y < 120)
      {
        cout<<"Look Up "<<endl;
        high_cmd_stop.head[0] = 0xFE;
        high_cmd_stop.head[1] = 0xEF;
        high_cmd_stop.levelFlag = HIGHLEVEL;
        high_cmd_stop.mode = 1;
        high_cmd_stop.gaitType = 0;
        high_cmd_stop.speedLevel = 0;
        high_cmd_stop.footRaiseHeight = 0;
        high_cmd_stop.bodyHeight = 0;
        high_cmd_stop.euler[0] = 0.0;
        high_cmd_stop.euler[1] = -0.3;
        high_cmd_stop.euler[2] = 0.0;
        high_cmd_stop.velocity[0] = 0.0f;
        high_cmd_stop.velocity[1] = 0.0f;
        high_cmd_stop.yawSpeed = 0.0f;
        high_cmd_stop.reserve = 0;
      } 

      pub_high_.publish(high_cmd_stop);
    }
    return BT::NodeStatus::SUCCESS;

  } else {
    ros::spinOnce();
    std::cout << "Ball Not Close" << std::endl;
    std::cout << "Approaching ball..." << std::endl;
    
    return BT::NodeStatus::FAILURE;
  }
}

//Approach ball action node

class ApproachBall : public BT::AsyncActionNode
{
public:
  explicit ApproachBall(const std::string &name) : BT::AsyncActionNode(name, {})
  {
  }

  void halt() override
  {
    _haltRequested = true;
  }

  BT::NodeStatus tick() override
  {
    if (_haltRequested)
    {
      return BT::NodeStatus::IDLE;
    }

    auto start = std::chrono::high_resolution_clock::now();

    while ((ros::ok()) && (!_haltRequested))
    {
      ros::spinOnce();
      ros::Time current_time = ros::Time::now();

      if (last_msg_.z > 30)
      {
        return BT::NodeStatus::SUCCESS;
      }

      unitree_legged_msgs::HighCmd high_cmd_ros;

      for (size_t i = 0; i < 3; i++)
      {
        // Move left
      if(last_msg_.x >= 0 && last_msg_.x <= 165){
        cout<<"Moving left "<<endl;

        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;
        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 1;
        high_cmd_ros.speedLevel = 0;
        high_cmd_ros.footRaiseHeight = 0.08;
        high_cmd_ros.bodyHeight = 0;
        high_cmd_ros.euler[0] = 0;
        high_cmd_ros.euler[1] = 0;
        high_cmd_ros.euler[2] = 0;
        high_cmd_ros.velocity[0] = 0.0f;
        high_cmd_ros.velocity[1] = 0.0f;
        high_cmd_ros.yawSpeed = 0.5f;
        high_cmd_ros.reserve = 0;
      }
      // Move forward
      else if(last_msg_.x > 165 && last_msg_.x < 330){
        ros::spinOnce();
        cout<<"Moving forward "<<endl;
      
        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;
        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 1;
        high_cmd_ros.speedLevel = 0;
        high_cmd_ros.footRaiseHeight = 0.08;
        high_cmd_ros.bodyHeight = 0;
        high_cmd_ros.euler[0] = 0;
        high_cmd_ros.euler[1] = 0;
        high_cmd_ros.euler[2] = 0;
        high_cmd_ros.velocity[0] = 0.2f;
        high_cmd_ros.velocity[1] = 0.0f;
        high_cmd_ros.yawSpeed = 0.0f;
        high_cmd_ros.reserve = 0;
      }
      // Move right
      else if(last_msg_.x >= 330 && last_msg_.x <= 460){
        ros::spinOnce();
        cout<<"Moving right "<<endl;

        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;
        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 1;
        high_cmd_ros.speedLevel = 0;
        high_cmd_ros.footRaiseHeight = 0.08;
        high_cmd_ros.bodyHeight = 0;
        high_cmd_ros.euler[0] = 0;
        high_cmd_ros.euler[1] = 0;
        high_cmd_ros.euler[2] = 0;
        high_cmd_ros.velocity[0] = 0.0f;
        high_cmd_ros.velocity[1] = 0.0f;
        high_cmd_ros.yawSpeed = -0.5f;
        high_cmd_ros.reserve = 0;
      }  
        pub_high_.publish(high_cmd_ros);
      }

      if ((current_time - last_message_time).toSec() > 0.5)
      {
        return BT::NodeStatus::FAILURE;
      }
    }
    
    return BT::NodeStatus::RUNNING;
  }

private:
  bool _haltRequested;
};

//Add a new behaviour



// Reset action node
class Reset : public BT::SyncActionNode
{
public:
  explicit Reset(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    ros::spinOnce();
    std::cout << "Reset Everything: " << std::endl;

    unitree_legged_msgs::HighCmd high_cmd_stop;

    for (size_t i = 0; i < 3; i++)
    {
      high_cmd_stop.head[0] = 0xFE;
      high_cmd_stop.head[1] = 0xEF;
      high_cmd_stop.levelFlag = HIGHLEVEL;
      high_cmd_stop.mode = 0;
      high_cmd_stop.gaitType = 0;
      high_cmd_stop.speedLevel = 0;
      high_cmd_stop.footRaiseHeight = 0;
      high_cmd_stop.bodyHeight = 0;
      high_cmd_stop.euler[0] = 0;
      high_cmd_stop.euler[1] = 0;
      high_cmd_stop.euler[2] = 0;
      high_cmd_stop.velocity[0] = 0.0f;
      high_cmd_stop.velocity[1] = 0.0f;
      high_cmd_stop.yawSpeed = 0.0f;
      high_cmd_stop.reserve = 0;

      pub_high_.publish(high_cmd_stop);  
    }

    return BT::NodeStatus::SUCCESS;
  }
};


static const char *xml_text = R"(
<root main_tree_to_execute = "MainTree" >
  <BehaviorTree ID="MainTree">
    <Fallback name="root_fallback">
      <Sequence name="robot_function">
        <Fallback name="find_ball_fallback">
          <BallFound name="ball_found"/>
          <SearchBall name="find_ball"/>
        </Fallback>
        <Fallback name="approach_ball_fallback">
          <ballClose name="ball_close"/>
          <ApproachBall name="approach_ball"/>
        </Fallback>
      </Sequence>
      <Reset name="reset_everything"/>
    </Fallback>
  </BehaviorTree>
</root>
)";

int main(int argc, char *argv[])
{
  // Initialize ROS
  ros::init(argc, argv, "behavior_tree");
  ros::NodeHandle nh;

  // Subscribe to the ball coordinates topic
  ros::Subscriber sub = nh.subscribe("/ball_coordinates", 10, &ballPositionCallback);

  // Publishers
  pub_high_ = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 10);

  // Register custom nodes in the factory
  BT::BehaviorTreeFactory factory;
  factory.registerSimpleCondition("BallFound", BallFound);
  factory.registerNodeType<SearchBall>("SearchBall");
  factory.registerSimpleCondition("ballClose", ballClose);
  factory.registerNodeType<ApproachBall>("ApproachBall");
  factory.registerNodeType<Reset>("Reset");

  unitree_legged_msgs::HighCmd high_cmd_stop;
  high_cmd_stop.head[0] = 0xFE;
  high_cmd_stop.head[1] = 0xEF;
  high_cmd_stop.levelFlag = 0xee;
  high_cmd_stop.mode = 0;
  high_cmd_stop.gaitType = 0;
  high_cmd_stop.speedLevel = 0;
  high_cmd_stop.footRaiseHeight = 0;
  high_cmd_stop.bodyHeight = 0;
  high_cmd_stop.euler[0] = 0;
  high_cmd_stop.euler[1] = 0;
  high_cmd_stop.euler[2] = 0;
  high_cmd_stop.velocity[0] = 0.0f;
  high_cmd_stop.velocity[1] = 0.0f;
  high_cmd_stop.yawSpeed = 0.0f;
  high_cmd_stop.reserve = 0;


  // Load behavior tree from XML text
  auto tree = factory.createTreeFromText(xml_text);

  // Variable which checks if joystick combinations have been pressed
  bool start_BT = false;

  // Run the behavior tree
  while (ros::ok()) {
    // Check rosparam current value to see if Point message is required 
    nh.getParam("/start_BT", start_BT);
    if (start_BT) tree.tickRoot();
    else pub_high_.publish(high_cmd_stop);
  }

  return 0;
}