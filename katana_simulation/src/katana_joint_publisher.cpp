#include <ros/ros.h>
//#include <string>
#include <sensor_msgs/JointState.h>
//#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

class JointMapper
{
public:
  JointMapper()
  {
    //Topic to publish: the name of the topic is used by robot_state_publisher
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    //Topic to subscribe: the topic is created in LasaKatana (written by Sina)
    joint_sub = n.subscribe("/Katana/Joint/Current", 10, &JointMapper::Callback, this);
  }

  void Callback(const std_msgs::Float32MultiArray& msg_joint)
  {
    // message declarations
    sensor_msgs::JointState joint_state;

    joint_state.name.resize(7);
    joint_state.position.resize(7);


    joint_state.name[0] ="katana_motor1_pan_joint";
    joint_state.name[1] ="katana_motor2_lift_joint";
    joint_state.name[2] ="katana_motor3_lift_joint";
    joint_state.name[3] ="katana_motor4_lift_joint";
    joint_state.name[4] ="katana_motor5_wrist_roll_joint";
    joint_state.name[5] ="katana_r_finger_joint";
    joint_state.name[6] ="katana_l_finger_joint";

    //map form one vector to the other
    joint_state.position[0] = msg_joint.data[0];
    joint_state.position[1] = msg_joint.data[1];
    joint_state.position[2] = msg_joint.data[2];
    joint_state.position[3] = msg_joint.data[3];
    joint_state.position[4] = msg_joint.data[4];
    joint_state.position[5] = 0;
    joint_state.position[6] = 0;

    

    joint_state.header.stamp = ros::Time::now();


    joint_pub.publish(joint_state);
  }

private:
  ros::NodeHandle n; 
  ros::Publisher joint_pub;
  ros::Subscriber joint_sub;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "JointMapper");

  //Create an object of class JointMapper to read the robot joint angles and send it to robot_state_publisher and rviz
  JointMapper jm_object;

  ros::spin();

  return 0;
}
