#include "pick_place_tasker.h"
PickPlaceTasker::PickPlaceTasker(ros::NodeHandle& n) : 
        nh_(n),
        pick_place_action_server_(nh_, "/pick_place_tasker/pick_place_action", \
                boost::bind(&PickPlaceTasker::pickPlaceActionServerCallback, this, _1), false)
{
        box_size_ = 0.05;
        pick_place_action_server_.start();
        ROS_WARN("Started service.....");
}

PickPlaceTasker::~PickPlaceTasker() {}

void PickPlaceTasker::pickPlaceActionServerCallback(const \
                                                pick_place_blocks::PickPlaceGoalConstPtr &goal)
{
        std::vector<geometry_msgs::Point> boxes;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        spawnBoxes(boxes, planning_scene_interface);
        ROS_INFO("Created two boxes at random locations, going to pick box0 and place it on box1");
        pickAndStack(boxes);
        pick_place_action_server_.setSucceeded();       
}


void PickPlaceTasker::pickAndStack(std::vector<geometry_msgs::Point>& boxes)
{
}


float PickPlaceTasker::getRandomFloat(float upper_limit, float lower_limit)
{
        return (static_cast<float>(rand())/static_cast<float>(RAND_MAX))* \ 
                                                (upper_limit - lower_limit) + lower_limit;
}

void PickPlaceTasker::generateRandomCoordinates(std::vector<geometry_msgs::Point>& boxes)
{
        float planner_limit_lower = 0.3;
        float planner_limit_upper = 0.5;
        srand(time(0));
        geometry_msgs::Point boxA, boxB;
        boxA.x = getRandomFloat(planner_limit_upper, planner_limit_lower);
        boxA.y = getRandomFloat(planner_limit_upper, planner_limit_lower);
        boxes.push_back(boxA);

        while(ros::ok())
        {
                boxB.x = getRandomFloat(planner_limit_upper, planner_limit_lower);
                boxB.y = getRandomFloat(planner_limit_upper, planner_limit_lower);
                // check for collision
                if(sqrt(pow(boxA.x - boxB.x, 2) + pow(boxA.y - boxB.y, 2)) > 2*box_size_) 
                {
                        boxes.push_back(boxB);
                        return;
                }
        }
}

void PickPlaceTasker::spawnBoxes(std::vector<geometry_msgs::Point>& boxes, \
                moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        
        collision_objects.resize(3);
        generateRandomCoordinates(boxes);
        
        collision_objects[0].id = "table";
        collision_objects[0].header.frame_id = "panda_link0";

        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[0].primitives[0].dimensions.resize(3);
        collision_objects[0].primitives[0].dimensions[0] = 0.6;
        collision_objects[0].primitives[0].dimensions[1] = 0.6;
        collision_objects[0].primitives[0].dimensions[2] = 0.2;
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0.5;
        collision_objects[0].primitive_poses[0].position.y = 0.3;
        collision_objects[0].primitive_poses[0].position.z = 0.2;
        collision_objects[0].primitive_poses[0].orientation.w = 1.0;
        collision_objects[0].operation = collision_objects[0].ADD;

        for(int i = 1; i < 3; i++)
        {
                collision_objects[i].id = "box" + std::to_string(i-1);
                collision_objects[i].header.frame_id = "panda_link0";

                collision_objects[i].primitives.resize(1);
                collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
                collision_objects[i].primitives[0].dimensions.resize(3);
                collision_objects[i].primitives[0].dimensions[0] = box_size_;
                collision_objects[i].primitives[0].dimensions[1] = box_size_;
                collision_objects[i].primitives[0].dimensions[2] = 0.05;

                collision_objects[i].primitive_poses.resize(1);
                

                collision_objects[i].primitive_poses[0].position.x = boxes[i-1].x;
                collision_objects[i].primitive_poses[0].position.y = boxes[i-1].y;
                collision_objects[i].primitive_poses[0].position.z = 0.35;
                collision_objects[i].primitive_poses[0].orientation.w = 1.0;

                collision_objects[i].operation = collision_objects[i].ADD;

        }

        planning_scene_interface.applyCollisionObjects(collision_objects);
}


int main(int argc, char **argv)
{
        ros::init(argc, argv, "pick_place_tasker");
        ros::NodeHandle nh;
        PickPlaceTasker picker(nh);

        ros::spin();
        return 0;
}
