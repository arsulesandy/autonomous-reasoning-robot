#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include <map>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <world_percept_assig4/DirectPose.h>


class DirectWorldInfo
{
private:

    std::string subs_topic_name_;        ///< gazebo model state topic name
    ros::Subscriber sub_gazebo_data_;    ///< Subscriber gazebo model_states
    std::map<std::string, geometry_msgs::Pose> last_poses_;
    std::map<std::string, ros::Time> last_stamp_;
    ros::ServiceServer srv_direct_pose_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher marker_pub_;

public:

    DirectWorldInfo(ros::NodeHandle& nh)
    : tf_listener_(tf_buffer_)
    {
        ROS_WARN_STREAM("Created world info");

        subs_topic_name_="/gazebo/model_states";

        // Create subscriber to receive the commanded turtle state. This state will be generated from a trajectory generator
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &DirectWorldInfo::topic_callback, this);

        ROS_INFO_STREAM("DirectWorldInfo subscribed to topic: " << subs_topic_name_);

        srv_direct_pose_ = nh.advertiseService(
            "direct_pose_srv",
            &DirectWorldInfo::service_callback,
            this);

        marker_pub_ = nh.advertise<visualization_msgs::Marker>(
            "direct_objects_markers", 1);
    };

    ~DirectWorldInfo()
    {

    };

private:

/**
   * @brief Callback function to receive the Gazebo Model State topic
   *
   * @param msg message with the current Gazebo model state
   */
  void topic_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
     static tf::TransformBroadcaster broadcaster;
    // Create a TF vector that stores the pose of the publish multiple TFs
    std::vector<geometry_msgs::TransformStamped> v_ts;

    // Get the current time
    ros::Time aux_time = ros::Time::now();

    // search new objects in the scene
    for (int i = 0; i < msg->name.size(); i++)
    {
        ///////////// TF broadcast

        geometry_msgs::TransformStamped ts;


        std::string model_name = msg->name[i];
        geometry_msgs::Pose pose = msg->pose[i];

        // Cache last known pose for service queries
        last_poses_[model_name] = pose;

        // TF object to populate our TF message
        tf2::Transform tf;

        tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
        tf.setRotation(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

        // Transform the TF object to TF message
        ts.transform = tf2::toMsg(tf);

        // Set the reference frame for the TF (parent link)
        //TODO: You need to also make some modification to the launch file.
        // You need to check which frame_id is defined in the launch file as the reference frame
        ts.header.frame_id = "world";
        // Set the name for the TF
        ts.child_frame_id = model_name + "_direct";
        // Set the time stamp for the message, enforcing monotonically increasing stamps per frame
        ros::Time desired_stamp = aux_time;
        ros::Time &prev_stamp = last_stamp_[ts.child_frame_id];  // creates entry if missing
        if (desired_stamp <= prev_stamp) {
            desired_stamp = prev_stamp + ros::Duration(1e-6);  // nudge forward
        }
        ts.header.stamp = desired_stamp;
        prev_stamp = desired_stamp;

        // Small log for first few objects so it doesn't spam
        if (i < 5) {
            ROS_INFO_STREAM_THROTTLE(2.0, "Publishing TF for model: " << model_name
                                        << " as frame: " << ts.child_frame_id);
        }

         //// To visualize objects in Rviz we need to publish its corresponding TF
        // Create TF msg
        v_ts.push_back(ts);

        ///////////// TF broadcast

        // --------- Task 4: publish marker for this object in robot frame ---------
        geometry_msgs::PoseStamped world_ps;
        world_ps.header.frame_id = "world";
        world_ps.header.stamp = aux_time;
        world_ps.pose = pose;

        geometry_msgs::PoseStamped robot_ps;
        std::string target_frame = "base_footprint";

        try
        {
            robot_ps = tf_buffer_.transform(world_ps, target_frame, ros::Duration(0.05));

            visualization_msgs::Marker m;
            m.header.frame_id = target_frame;
            m.header.stamp = aux_time;
            m.ns = "direct_objects";
            m.id = i;  // one id per object in this message
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;

            m.pose = robot_ps.pose;

            m.scale.x = 0.1;
            m.scale.y = 0.1;
            m.scale.z = 0.1;

            m.color.r = 0.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;

            m.lifetime = ros::Duration(0.2);

            marker_pub_.publish(m);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_STREAM_THROTTLE(2.0,
                "Marker transform failed for " << model_name << " : " << ex.what());
        }
        // -------------------------------------------------------------------------

    }//for msg size

     //Once all the information of the TFs is obtained, then we broadcast this data to Rviz
    if (!v_ts.empty()) {
        broadcaster.sendTransform(v_ts);
        ROS_INFO_STREAM_THROTTLE(1.0, "Broadcasted " << v_ts.size() << " TFs from world.");
    }

  } // callback


  bool service_callback(
      world_percept_assig4::DirectPose::Request &req,
      world_percept_assig4::DirectPose::Response &res)
  {
      const std::string &name = req.object_name;

      std::map<std::string, geometry_msgs::Pose>::const_iterator it = last_poses_.find(name);
      if (it == last_poses_.end())
      {
          res.success = false;
          res.message = "Object not found: " + name;
          return true;
      }

      // Pose in world frame (from Gazebo)
      geometry_msgs::Pose world_pose = it->second;

      geometry_msgs::PoseStamped world_ps;
      world_ps.header.frame_id = "world";
      world_ps.header.stamp = ros::Time::now();
      world_ps.pose = world_pose;

      geometry_msgs::PoseStamped robot_ps;

      std::string target_frame = "base_footprint";

      try
      {
          robot_ps = tf_buffer_.transform(world_ps, target_frame, ros::Duration(0.5));
      }
      catch (tf2::TransformException &ex)
      {
          ROS_WARN_STREAM("direct_pose_srv: TF transform failed from world to "
                          << target_frame << " for object " << name << ": " << ex.what());
          res.pose = world_pose;
          res.success = false;
          res.message = "TF transform failed; returning world pose";
          return true;
      }

      res.pose = robot_ps.pose;
      res.success = true;
      res.message = "OK";
      return true;
  }

}; // Class

int main(int argc, char** argv)
{
    ros::init(argc, argv, "direct_percept_node");
    ros::NodeHandle nh;

    DirectWorldInfo myDirectWorld(nh);

    ros::spin();

    return 0;
}
