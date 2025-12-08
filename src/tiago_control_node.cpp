#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>

#include <world_percept_assig4/GetSceneObjectList.h>
#include <world_percept_assig4/ComputeTiagoTwist.h>

class TiagoController
{
public:
    explicit TiagoController(ros::NodeHandle &nh)
    : nh_(nh),
      have_tiago_pose_(false),
      have_target_pose_(false),
      target_name_("table"),
      last_query_(ros::Time(0))
    {
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("target_name", target_name_, target_name_);

        sub_ = nh_.subscribe("/gazebo/model_states", 10, &TiagoController::modelStatesCb, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/key_vel", 1);
        scene_client_ = nh_.serviceClient<world_percept_assig4::GetSceneObjectList>("get_scene_object_list");
        compute_srv_ = nh_.advertiseService("compute_target_twist", &TiagoController::computeServiceCb, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher cmd_pub_;
    ros::ServiceClient scene_client_;
    ros::ServiceServer compute_srv_;

    geometry_msgs::Pose tiago_pose_;
    geometry_msgs::Pose target_pose_;
    bool have_tiago_pose_;
    bool have_target_pose_;
    std::string target_name_;
    geometry_msgs::Twist last_cmd_;
    ros::Time last_query_;

    static Eigen::Matrix2d q2Rot2D(const geometry_msgs::Quaternion &quaternion)
    {
        Eigen::Quaterniond eigenQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        return eigenQuaternion.toRotationMatrix().block<2,2>(0,0);
    }

    bool updateTargetPose(const std::string &name)
    {
        world_percept_assig4::GetSceneObjectList srv;
        srv.request.object_name = name;

        if (!scene_client_.call(srv))
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "Failed to call get_scene_object_list");
            have_target_pose_ = false;
            return false;
        }

        if (!srv.response.obj_found || srv.response.objects.pose.empty())
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "Target object [" << name << "] not found");
            have_target_pose_ = false;
            return false;
        }

        target_pose_ = srv.response.objects.pose.front();
        have_target_pose_ = true;
        return true;
    }

    geometry_msgs::Twist computeCommand() const
    {
        geometry_msgs::Twist tiago_twist_cmd;

        if (!have_tiago_pose_ || !have_target_pose_)
        {
            return tiago_twist_cmd;
        }

        // Compute control in robot frame using hints from the assignment slides
        Eigen::Vector2d Dpose_w;
        Dpose_w << target_pose_.position.x - tiago_pose_.position.x,
                   target_pose_.position.y - tiago_pose_.position.y;

        Eigen::Matrix2d Rtiago_w = q2Rot2D(tiago_pose_.orientation);
        Eigen::Matrix2d Rw_tiago = Rtiago_w.inverse();

        Eigen::Vector2d Dpose_tiago = Rw_tiago * Dpose_w;
        double d = (Dpose_tiago.norm() <= 1.3) ? 0.0 : Dpose_tiago.norm();
        double theta = std::atan2(Dpose_tiago(1), Dpose_tiago(0));

        double Kwz = 1.1;
        double Kvx = 0.1;

        tiago_twist_cmd.linear.x = Kvx * d;
        tiago_twist_cmd.angular.z = Kwz * theta;
        return tiago_twist_cmd;
    }

    void modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        // Locate Tiago pose in the incoming model state list
        auto it = std::find(msg->name.begin(), msg->name.end(), "tiago");
        if (it == msg->name.end())
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "Tiago not found in /gazebo/model_states");
            have_tiago_pose_ = false;
            return;
        }

        const size_t index = static_cast<size_t>(std::distance(msg->name.begin(), it));
        tiago_pose_ = msg->pose.at(index);
        have_tiago_pose_ = true;

        // Throttle calls to the map generator service
        if ((ros::Time::now() - last_query_) < ros::Duration(0.25))
        {
            return;
        }

        last_query_ = ros::Time::now();

        if (!updateTargetPose(target_name_))
        {
            return;
        }

        geometry_msgs::Twist cmd = computeCommand();
        last_cmd_ = cmd;
        cmd_pub_.publish(cmd);
    }

    bool computeServiceCb(world_percept_assig4::ComputeTiagoTwist::Request &req,
                          world_percept_assig4::ComputeTiagoTwist::Response &res)
    {
        const std::string &requested_target = req.target_name.empty() ? target_name_ : req.target_name;

        if (!have_tiago_pose_)
        {
            res.success = false;
            res.message = "Tiago pose not available yet";
            return true;
        }

        if (!updateTargetPose(requested_target))
        {
            res.success = false;
            res.message = "Target object not available";
            return true;
        }

        geometry_msgs::Twist cmd = computeCommand();
        last_cmd_ = cmd;
        target_name_ = requested_target;

        cmd_pub_.publish(cmd);
        res.cmd_vel = cmd;
        res.success = true;
        res.message = "Command computed";
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tiago_control_node");
    ros::NodeHandle nh;

    TiagoController controller(nh);
    ros::spin();
    return 0;
}
