#include "ros/ros.h"
#include "knowledge_msgs/GraspIndividual.h"
#include <motion_msgs/MovingCommandAction.h>
#include <string>
#include <iostream>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "test_client");
   
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<knowledge_msgs::GraspIndividual>("/knowledge_grasp/knowledge_grasp");

     knowledge_msgs::GraspIndividual srv;
    std::string objectLabel;
    n.getParam("objectLabel", objectLabel);

     srv.request.object_label = objectLabel;

    ROS_INFO_STREAM ("TRY TO CALL GRASP SERVICE.");

     if(client.call(srv)){
         ROS_INFO_STREAM ("SUCCESSFULLY CALLED GRASP SERVICE.");

         ROS_INFO_STREAM ("SENDING GOAL TO MOVINGCOMMANDACTION.");
         actionlib::SimpleActionClient<motion_msgs::MovingCommandAction> ac("moving", true);

         ROS_INFO_STREAM("WAITING FOR THE ACTION SERVER TO START.");
         ac.waitForServer();

         // Sending goal for grasping object with left arm
         ROS_INFO_STREAM("SENDING ACTION GRASP GOAL.");
         motion_msgs::MovingCommandGoal goal;
         goal.command = motion_msgs::MovingCommandGoal::GRASP_LEFT_ARM;
         goal.goal_poses = srv.response.grasp_pose_array;
         goal.direction_key = srv.response.direction_key;
         goal.grasped_object_label = objectLabel;
         goal.force = -1;

         ac.sendGoal(goal);

         ROS_INFO_STREAM("WAIT FOR THE ACTION TO RETURN.");
         bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

         if (finished_before_timeout)
         {
             actionlib::SimpleClientGoalState state = ac.getState();
             ROS_INFO ("ACTION FINISHED: %s",state.toString().c_str());
         }
         else
             ROS_INFO("ACTION DID NOT FINISH BEFORE TIMEOUT.");


         // Sending goal for move to grasped-object-home-pos with left arm
         ROS_INFO_STREAM("SENDING MOVE TO GRASPED OBJECT HOME POSITION.");
         motion_msgs::MovingCommandGoal goalCarryPos;
         goalCarryPos.command = motion_msgs::MovingCommandGoal::MOVE_CARRY_POSE_LEFT;

         ac.sendGoal(goalCarryPos);

         ROS_INFO_STREAM("WAIT FOR THE ACTION TO RETURN.");
         finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

         if (finished_before_timeout)
         {
             actionlib::SimpleClientGoalState state = ac.getState();
             ROS_INFO ("ACTION FINISHED: %s",state.toString().c_str());
         }
         else
             ROS_INFO("ACTION DID NOT FINISH BEFORE TIMEOUT.");


     } else{
        ROS_ERROR("COULDN'T CALL GRASP SERVICE, ABORTING!");
         return 0;
     }



     return 0;
}