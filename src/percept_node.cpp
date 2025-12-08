#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>
#include <world_percept_assig4/UpdateObjectList.h>
#include <world_percept_assig4/SetInitTiagoPose.h>

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <sstream>

class WorldInfo
{
private:

    std::string subs_topic_name_;        ///< gazebo model state topic name
    std::string srv_update_obj_name_;        ///< name of the service provided by the map generator node
    ros::Subscriber sub_gazebo_data_;     ///< Subscriber gazebo model_states

    std::vector<std::string> v_seen_obj_;  ///< List of objects seen by the robot and sent to the map generator node

    ros::ServiceClient client_learning_; ///< Client to request the object list update in the learning node

    bool use_ground_truth_;

public:

    WorldInfo(ros::NodeHandle& nh)
    {
        ROS_WARN_STREAM("Created world info");

        ros::NodeHandle pnh("~");
        pnh.param("use_ground_truth", use_ground_truth_, false);

        // This objects will not be sent to the Map generator node
        v_seen_obj_.push_back("tiago");
        v_seen_obj_.push_back("ground_plane");

        subs_topic_name_="/gazebo/model_states";

        // create client and wait until service is advertised
        srv_update_obj_name_="update_object_list";

        client_learning_ = nh.serviceClient<world_percept_assig4::UpdateObjectList>(srv_update_obj_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_update_obj_name_.c_str());
        bool service_found = ros::service::waitForService(srv_update_obj_name_, ros::Duration(30.0)); // You can adjust the timeout as needed

        if(!service_found)
        {
            std::ostringstream err;
            err << "Service " << srv_update_obj_name_ << " was not advertised. Cannot continue.";
            ROS_FATAL("%s", err.str().c_str());
            throw std::runtime_error(err.str());
        }

        ROS_INFO_STREAM("Connected to service: "<<srv_update_obj_name_);

        if (use_ground_truth_)
        {
            sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &WorldInfo::topic_callback, this);
            ROS_WARN_STREAM("percept_node using /gazebo/model_states ground truth (use_ground_truth:=true).");
        }
        else
        {
            ROS_INFO_STREAM("percept_node idle (use_ground_truth:=false); learning_node handles sensor-driven detections.");
        }
    };

    ~WorldInfo()
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

    //Get robot Pose
    geometry_msgs::Pose tiago_pose;
    // Search for tiago pose
   auto it = std::find( msg->name.begin(),  msg->name.end(), "tiago");
    if (it != msg->name.end())
    {
        // Calculate the index
        int index = std::distance(msg->name.begin(), it);
        tiago_pose=msg->pose.at(index);
    }

    // search new objects in the scene
    for (int i = 0; i < msg->name.size(); i++)
    {
        // Get obj position
        // Get tiago position
        // Compare distances, if within range then check the v_seen_list
        // if the obj is not in the list add it and send it to the srv

        // Get object pose
        geometry_msgs::Pose obj_pose = msg->pose.at(i);

        // get distance from tiago to obj[i]
        double dx = tiago_pose.position.x - obj_pose.position.x;
        double dy = tiago_pose.position.y - obj_pose.position.y;
        double d = std::sqrt(dx*dx + dy*dy);

        //IF the robot is closer to the seen objects, then request the service
        if (d<1.1)
        {
            std::string s = msg->name.at(i);
            // Search for the obj name in the seen_list
            auto it = std::find(v_seen_obj_.begin(), v_seen_obj_.end(), s);

            // If the obj name is not found in the seen vector, this means that the robot has seen a new object for the first time and it should add it to the seen vector and call the service update_object_list
            if (it == v_seen_obj_.end()) {

                world_percept_assig4::UpdateObjectList srv;

                srv.request.object_name = s;
                srv.request.object_pose = obj_pose;

                bool map_updated = false;

                if (client_learning_.call(srv))
                {
                    ROS_INFO_STREAM("Object List Updated?: "<< (int)srv.response.confirmation);

                    if(srv.response.confirmation)
                    {
                        v_seen_obj_.push_back(s);

                        ROS_INFO_STREAM("Object ["<<s<<"] added to the list");
                        map_updated = true;
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("Failed to call service "<<srv_update_obj_name_);
                }
            }
        } //if d
    }//for msg size

    //If you want to print the objects that the robot has seen so far, just uncomment the for
    for (size_t i = 2; i < v_seen_obj_.size(); i++)
    {
        ROS_INFO_STREAM("["<<i<<"]: "<<v_seen_obj_.at(i));
    }

  } // callback



}; // Class

int main(int argc, char** argv)
{
    ros::init(argc, argv, "percept_node");
    ros::NodeHandle nh;

    WorldInfo myPercept(nh);

    ros::spin();

    return 0;
}
