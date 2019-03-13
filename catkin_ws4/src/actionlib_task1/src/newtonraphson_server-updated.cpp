
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_task1/NewtonRaphsonAction.h>
#include <cmath>
#include <iostream>

class NewtonRaphsonAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_task1::NewtonRaphsonAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_task1::NewtonRaphsonFeedback feedback_;
  actionlib_task1::NewtonRaphsonResult result_;

public:

  NewtonRaphsonAction(std::string name) :
    as_(nh_, name, boost::bind(&NewtonRaphsonAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~NewtonRaphsonAction(void)
  {
  }

  void executeCB(const actionlib_task1::NewtonRaphsonGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the newtonraphson ratio
    //feedback_.ratio.clear();
    //feedback_.ratio.push_back(0);
    //feedback_.ratio.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating newtonraphson ratio : x = %lf", action_name_.c_str(), goal->x);

    // start executing the action
       double x = goal -> x ;
    
       double f = pow(x,3) − 5*x + 13;
    
       double f1 = 3*pow(x,2) - 5;
    
       double ratio =​ (f)/(f1);

      // check that preempt has not been requested by the client      if (as_.isPreemptRequested() || !ros::ok())
      /*{
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }*/
      feedback_.ratio = ratio;
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the ratio is computed at 1 Hz for demonstration purposes
      r.sleep();
    

    if(success)
    {
      result_.ratio = feedback_.ratio;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "newtonraphson");

  NewtonRaphsonAction newtonraphson("newtonraphson");
  ros::spin();

  return 0;
}