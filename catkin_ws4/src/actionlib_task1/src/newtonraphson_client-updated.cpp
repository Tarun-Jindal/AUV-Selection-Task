
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_task1/NewtonRaphsonAction.h>
#include <iostream>

using namespace actionlib_task1;

double ratio;

typedef actionlib::SimpleActionClient<NewtonRaphsonAction> Client;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const NewtonRaphsonResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
 // ROS_INFO("Answer: %i", result->ratio.back());
  ROS_INFO("Answer: %lf", result->ratio);
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal is active");
}

// Called every time feedback is received for the goal
void feedbackCb(const NewtonRaphsonFeedbackConstPtr& feedback)
{
  //ROS_INFO("Got Feedback of length %lf", feedback->ratio.size());
  ROS_INFO("Got Feedback of length %lf", feedback->ratio);
  ratio = feedback->ratio;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_newtonraphson_ratio");

  // Create the action client
  Client ac("newtonraphson", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  // Send Goal
  NewtonRaphsonGoal goal;
  goal.x = 10;
  ratio = 3;
  while(ratio > 0.000001)
  {
    goal.x = goal.x - ratio;
  
  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
 //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  
}
  
ROS_INFO("Final root is  %lf ", goal.x);
  

ros::spin();
 
 

  return 0;
}