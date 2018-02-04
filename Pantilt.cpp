#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asr_flir_ptu_driver/State.h"
#include <sensor_msgs/JointState.h>

//double headset_pan = 0;
//double headset_tilt = 0;
//double tolerance = 0.1;
asr_flir_ptu_driver::State movement;
ros::Publisher state_pub;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
/*void rotationCallback(const asr_flir_ptu_driver::State::ConstPtr& msg)
{
  headset_pan = std::abs(headset_pan - msg->state.position[0]);
  headset_tilt = std::abs(headset_pan - msg->state.position[1]);
  movement.state.position[0] = headset_pan;
  movement.state.position[1] = headset_tilt;
  ROS_INFO("Entra");
}*/

void rotationCallback(const asr_flir_ptu_driver::State::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f][%f]", msg->state.position[0], msg->state.position[1]);
  ROS_INFO("Posicion actual: [%f][%f]", movement.state.position[0], movement.state.position[1]);
  asr_flir_ptu_driver::State movement_goal;
  movement_goal.state.position.push_back((movement.state.position[0] + msg->state.position[0])*180.0/3.141592);
  movement_goal.state.position.push_back((movement.state.position[1] + msg->state.position[1])*180.0/3.141592);
  movement_goal.state.velocity.push_back(1.0);
  movement_goal.state.velocity.push_back(1.0);

  ROS_INFO("Goal: [%f][%f]", movement_goal.state.position[0], movement_goal.state.position[1]);

  state_pub.publish(movement_goal);
  
}

void pantiltCallback(const asr_flir_ptu_driver::State::ConstPtr& msg)
{ 

  movement.state.position[0] = msg->state.position[0];
  movement.state.position[1] = msg->state.position[1];
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "Pantilt_subscriber");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  movement.state.position.push_back(0.0);
  movement.state.position.push_back(0.0);
// %Tag(SUBSCRIBER)%
  state_pub = n.advertise<asr_flir_ptu_driver::State>("asr_flir_ptu_driver/state_cmd", 1);
  ros::Subscriber sub = n.subscribe("Rotacion", 1, rotationCallback);
  ros::Subscriber state_sub = n.subscribe<asr_flir_ptu_driver::State>("asr_flir_ptu_driver/ptu_state", 1, pantiltCallback);
  //state_pub.publish(movement);

// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%