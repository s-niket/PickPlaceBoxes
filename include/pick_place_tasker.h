#include <iostream>
#include <time.h>
#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "pick_place_blocks/BlockSpawn.h"
#include "pick_place_blocks/PickPlaceAction.h"
#include <actionlib/server/simple_action_server.h>
#include <moveit_msgs/Grasp.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#ifndef PICK_PLACE_TASKER_H_
#define PICK_PLACE_TASKER_H_

class PickPlaceTasker
{
private:
        // ros node handler
        ros::NodeHandle nh_;

        // size of the boxes to be spawned
        float box_size_;
public:
        /**
         * @brief Construct a new Pick Place Tasker object
         * 
         * @param n ros node handler
         */
        PickPlaceTasker(ros::NodeHandle& n);

        /**
         * @brief Destroy the Pick Place Tasker object
         * 
         */
        ~PickPlaceTasker();

        /**
         * @brief method to generate a random float number between the upper and the lower limit
         * 
         * @param upper_limit upper limit
         * @param lower_limit lower limit
         * @return float random float
         */
        float getRandomFloat(float upper_limit, float lower_limit);

        /**
         * @brief method to generate random positions for the two boxes 
         * 
         * @param boxes vector of positions
         */
        void generateRandomCoordinates(std::vector<geometry_msgs::Point>& boxes);

        /**
         * @brief methood to spawn two boxes at random locations in the current planning
         *        scene
         * 
         * @param boxes box positions
         * @param planning_scene_interface planning scene object reference
         */
        void spawnBoxes(std::vector<geometry_msgs::Point>& boxes, \
                moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
};
#endif // PICK_PLACE_TASKER_H_