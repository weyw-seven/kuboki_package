#include <my_planner/my_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav_core::BaseGlobalPlanner
)

namespace my_planner {

    MyPlanner::MyPlanner()
            : costmap_ros_(NULL), initialized_(false) {}

    MyPlanner::MyPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
            : costmap_ros_(NULL), initialized_(false) {
        initialize(name, costmap_ros);
    }

    void MyPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
        if (!initialized_) {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            global_frame_ = costmap_ros_->getGlobalFrameID();

            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

            private_nh.param("step_size", step_size_, costmap_->getResolution());
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
            world_model_ = new base_local_planner::CostmapModel(*costmap_);
            outfile.open("/home/hsy/mypath.txt"); // TODO 绝对路径
            initialized_ = true;
        } else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    //we need to take the footprint of the robot into account when we calculate cost to obstacles
    double MyPlanner::footprintCost(double x_i, double y_i, double theta_i) {
        if (!initialized_) {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return -1.0;
        }

        std::vector <geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
        //if we have no footprint... do nothing
        if (footprint.size() < 3)
            return -1.0;

        //check if the footprint is legal
        double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
        return footprint_cost;
    }

    /* TODO
     * below the 'makePlan' part is up to you, the whole contents of this method is about your algorithm.
        The 'start' and 'goal' parameters are used to get initial location and target location, respectively.
        In this illustrative implementation, the plan vector is cleared, and is initiated with the start location (plan.push_back(start)) after.
        This planned path will then be sent to the 'move_base' global planner module which will publish it through ROS topic 'nav_msgs/Path',
        which will then be received by the local planner module.
    */
    bool MyPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                             const geometry_msgs::PoseStamped &goal, std::vector <geometry_msgs::PoseStamped> &plan) {

        if (!initialized_) {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }
        //起点位置
        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
                  goal.pose.position.x, goal.pose.position.y);

        plan.clear();
        //使用的地图数据
        costmap_ = costmap_ros_->getCostmap();



        //goal：终点位置
        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
            ROS_ERROR(
                    "This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                    costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        const double start_yaw = tf2::getYaw(start.pose.orientation);
        const double goal_yaw = tf2::getYaw(goal.pose.orientation);

        double goal_x = goal.pose.position.x;
        double goal_y = goal.pose.position.y;
        double start_x = start.pose.position.x;
        double start_y = start.pose.position.y;

        double diff_x = goal_x - start_x;
        double diff_y = goal_y - start_y;
        double diff_yaw = angles::normalize_angle(goal_yaw - start_yaw);

        //target_x, target_y为待求目标点位置
        double target_x = goal_x;
        double target_y = goal_y;
        double target_yaw = goal_yaw;

        bool done = false;
        double scale = 1.0;
        double dScale = 0.01;

        // 以下为示例代码，TODO，根据地图信息得到目标点 这里给出了一个点的示例，也可以同时得出多个路径点
        //--------
        while (!done) {
            if (scale < 0) {
                target_x = start_x;
                target_y = start_y;
                target_yaw = start_yaw;
                ROS_WARN("The planner could not find a valid plan for this goal");
                break;
            }
            target_x = start_x + scale * diff_x;
            target_y = start_y + scale * diff_y;
            target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);

            double footprint_cost = footprintCost(target_x, target_y, target_yaw);
            if (footprint_cost >= 0) {
                done = true;
            }
            scale -= dScale;
        }
        //-------

        plan.push_back(start);
        geometry_msgs::PoseStamped new_goal = goal;
        tf2::Quaternion goal_quat;
        goal_quat.setRPY(0, 0, target_yaw);

        new_goal.pose.position.x = target_x;
        new_goal.pose.position.y = target_y;

        new_goal.pose.orientation.x = goal_quat.x();
        new_goal.pose.orientation.y = goal_quat.y();
        new_goal.pose.orientation.z = goal_quat.z();
        new_goal.pose.orientation.w = goal_quat.w();

        plan.push_back(new_goal);
        publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
        {
            outfile << fixed;
            outfile << setprecision(6) << goal.header.stamp.sec << "." << goal.header.stamp.nsec
                    << ": " << start_x << " " << start_y << " " << start_yaw << ", " << goal_x << " " << goal_y << " "
                    << goal_yaw << endl << "path: " << endl;
            for (size_t i = 0; i < plan.size(); i++) {
                geometry_msgs::PoseStamped newplan = plan[i];
                const double yaw = tf2::getYaw(newplan.pose.orientation);
                outfile << new_goal.pose.position.x << " " << new_goal.pose.position.y << " " << yaw;
                if (i < plan.size() - 1)
                    outfile << ", ";
            }
            outfile << endl;
        }

        return (done);
    }

    void MyPlanner::publishPlan(const std::vector <geometry_msgs::PoseStamped> &path, double r, double g, double b,
                                double a) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        if (path.empty()) {
            //still set a valid frame so visualization won't hit transform issues
            gui_path.header.frame_id = global_frame_;
            gui_path.header.stamp = ros::Time::now();
        } else {
            gui_path.header.frame_id = path[0].header.frame_id;
            gui_path.header.stamp = path[0].header.stamp;
        }

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }

};
