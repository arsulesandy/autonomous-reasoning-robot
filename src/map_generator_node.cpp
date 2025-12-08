#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <world_percept_assig4/UpdateObjectList.h>
#include <world_percept_assig4/SetInitTiagoPose.h>
#include <world_percept_assig4/GetSceneObjectList.h>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>

class MapGenerator
{
private:


    std::string srv_update_obj_name_;        ///< service name for new obj
    ros::ServiceServer update_obj_list_srv_; // Advertise service to update obj list
    std::vector<std::pair<std::string, geometry_msgs::Pose>> map_objs_; ///< List of objects in the scene

    ros::Timer tf_timer_;                 ///< Timer to run a parallel process
    tf::TransformBroadcaster broadcaster_; ///< TF broadcaster variable

    ros::ServiceServer get_scene_obj_srv_; // Advertise service to send the pose of a target object in the scene
    std::string srv_get_scene_name_;       ///< service name to send a target object in the scene

    std::map<std::string, int> name2id; ///< Map object names to index in the object list
    std::map<int, std::string> id2name; ///< Map index in the object list to object names

public:

    MapGenerator(ros::NodeHandle& nh)
    {
        ROS_WARN_STREAM("Created world map");

        srv_update_obj_name_="update_object_list";

        update_obj_list_srv_ = nh.advertiseService(srv_update_obj_name_, &MapGenerator::srv_update_callback, this);

        srv_get_scene_name_ = "get_scene_object_list";
        get_scene_obj_srv_ = nh.advertiseService(srv_get_scene_name_, &MapGenerator::srv_get_scene_obj_callback, this);

        // publish the current position
        tf_timer_ = nh.createTimer(ros::Duration(1), &MapGenerator::tf_timer_callback, this);


    };

    ~MapGenerator()
    {

    };

private:

/**
   * @brief Callback function for the service that adds objects to the map_objects list
   *
   * @param Request requested object to be added to the list
   * @param Respose response from the service when the object has been added (true/false)
*/
bool srv_update_callback(world_percept_assig4::UpdateObjectList::Request  &req,
         world_percept_assig4::UpdateObjectList::Response &res)
{
    ROS_INFO_STREAM("Got new object: "<<req.object_name);
    ROS_INFO_STREAM("Object Pose: "<<req.object_pose);

    auto it = name2id.find(req.object_name);
    if (it == name2id.end())
    {
        map_objs_.push_back(std::make_pair(req.object_name, req.object_pose));
        const int idx = static_cast<int>(map_objs_.size() - 1);
        name2id[req.object_name] = idx;
        id2name[idx] = req.object_name;
        ROS_INFO_STREAM("Object [" << req.object_name << "] inserted at slot " << idx);
    }
    else
    {
        map_objs_.at(it->second).second = req.object_pose;
        ROS_INFO_STREAM("Object [" << req.object_name << "] pose updated.");
    }

    res.confirmation = true;
    return res.confirmation;
}

/**
     * @brief Callback function for the service that sends the info of a target object in the scene. This service could be use, for example, by the controller node to find the target position
     *
     * @param Request name of the requested object. Use "all" to get the list of all the available objects.
     * @param Respose response is a list of objects with name and pose.
     */
    bool srv_get_scene_obj_callback(world_percept_assig4::GetSceneObjectList::Request &req,
                                    world_percept_assig4::GetSceneObjectList::Response &res)
    {
        ROS_INFO_STREAM("Target object: " << req.object_name);

        // If the requested object name is "all", we should send all the available objects

        if (req.object_name == "all")
        {
            geometry_msgs::Twist zero_twist;

            res.objects.name.clear();
            res.objects.pose.clear();
            res.objects.twist.clear();

            for (const auto &entry : map_objs_)
            {
                res.objects.name.push_back(entry.first);
                res.objects.pose.push_back(entry.second);
                res.objects.twist.push_back(zero_twist);
            }

            res.obj_found = true;
            return res.obj_found;
        }

        // Verify if the target name exists in the list
        auto it = name2id.find(req.object_name);

        if (it != name2id.end())
        {
            // The target object name is in the list
            // Get the index in the pair list
            int target_idx = name2id[req.object_name];

            geometry_msgs::Twist zero_twist;
            res.objects.name = {map_objs_.at(target_idx).first};
            res.objects.pose = {map_objs_.at(target_idx).second};
            res.objects.twist = {zero_twist};
            res.obj_found = true;
        }
        else
        {
            // target object is not in the list
            res.obj_found = false;

            std::stringstream s;

            s<<"The target object [" << req.object_name << "] is not in the list. \nAvailable Objects";

            // copy all the pairs (names,poses) into the object list
            for (auto &&pairs : map_objs_)
            {
                s<<pairs.first<<"\n";
            }

            res.message = s.str();

            ROS_ERROR_STREAM("The target object [" << req.object_name << "] is not in the list");
        }

        return res.obj_found;
    }



void tf_timer_callback(const ros::TimerEvent& e)
{
    std::vector<geometry_msgs::TransformStamped> v_ts;

    // Get the current time
    ros::Time aux_time = ros::Time::now();

    for (size_t i = 0; i < map_objs_.size(); i++)
    {
        geometry_msgs::TransformStamped ts;

        const std::string& object_name = map_objs_.at(i).first;
        const geometry_msgs::Pose& obj_pose = map_objs_.at(i).second;

        // TF object to populate our TF message
        tf2::Transform tf;

        tf.setOrigin(tf2::Vector3(obj_pose.position.x, obj_pose.position.y, obj_pose.position.z));
        tf.setRotation(tf2::Quaternion(obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w));

        // Transform the TF object to TF message
        ts.transform = tf2::toMsg(tf);

        // Set the reference frame for the TF (parent link)
        ts.header.frame_id = "world";
        // Set the time stamp for the message
        ts.header.stamp = aux_time;
        // Set the name for the TF
        ts.child_frame_id = object_name;

         //// To visualize objects in Rviz we need to publish its corresponding TF
        // Create TF msg
        v_ts.push_back(ts);
    }

    //Once all the information of the TFs is obtained, then we broadcast this data to Rviz
    broadcaster_.sendTransform(v_ts);

}

};//class MapGenerator

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_generator_node");
    ros::NodeHandle nh;

    MapGenerator myMapGenerator(nh);

    ros::spin();

    return 0;
}
