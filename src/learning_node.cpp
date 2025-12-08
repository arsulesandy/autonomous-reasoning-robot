#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <world_percept_assig4/UpdateObjectList.h>
#include <world_percept_assig4/GetSceneObjectList.h>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>

class LearningNode
{
public:
    explicit LearningNode(ros::NodeHandle &nh)
        : nh_(nh),
          it_(nh_),
          tf_listener_(tf_buffer_),
          prolog_ready_(false),
          latest_obstacle_distance_(std::numeric_limits<double>::infinity())
    {
        nh_.param("obstacle_stop_threshold", obstacle_threshold_, 0.9);
        nh_.param("vision/target_name", vision_target_name_, std::string("red_blob"));
        nh_.param("vision/assumed_depth", assumed_depth_, 1.2);
        nh_.param("vision/camera_fov", camera_fov_rad_, 1.047);
        nh_.param("vision/image_topic", image_topic_, std::string("head_front_camera/rgb/image_raw"));

        support_labels_.insert("floor");

        update_srv_ = nh_.advertiseService("update_object_list", &LearningNode::updateCallback, this);
        scene_srv_ = nh_.advertiseService("get_scene_object_list", &LearningNode::sceneCallback, this);
        report_srv_ = nh_.advertiseService("report_learning_metrics", &LearningNode::reportCallback, this);

        reasoning_client_ = nh_.serviceClient<world_percept_assig4::UpdateObjectList>("assert_knowledge");

        if (pl_.waitForServer())
        {
            pl_ = PrologClient("/rosprolog", true);
            prolog_ready_ = true;
        }
        else
        {
            ROS_WARN_STREAM("rosprolog server is not available; reasoning predicates will be skipped");
        }

        laser_sub_ = nh_.subscribe("/scan", 1, &LearningNode::laserCallback, this);
        obstacle_pub_ = nh_.advertise<std_msgs::Float32>("learning/obstacle_distance", 1, true);
        metrics_pub_ = nh_.advertise<std_msgs::String>("learning/confusion_json", 1, true);

        image_sub_ = it_.subscribe(image_topic_, 1, &LearningNode::imageCallback, this);

        ROS_INFO_STREAM("learning_node ready; listening for perception updates");
    }

private:
    struct Stats
    {
        std::map<std::string, int> surface_counts;
        int total{0};
    };

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    ros::ServiceServer update_srv_;
    ros::ServiceServer scene_srv_;
    ros::ServiceServer report_srv_;

    ros::Subscriber laser_sub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher metrics_pub_;
    image_transport::Subscriber image_sub_;

    ros::ServiceClient reasoning_client_;
    PrologClient pl_;
    bool prolog_ready_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::map<std::string, geometry_msgs::Pose> objects_;
    std::map<std::string, geometry_msgs::Pose> supports_;
    std::map<std::string, Stats> learned_stats_;
    std::map<std::string, std::map<std::string, int>> confusion_; // actual -> predicted counts
    std::set<std::string> support_labels_;
    std::set<std::string> asserted_classes_;
    std::string best_surface_buffer_;

    double obstacle_threshold_{0.9};
    double latest_obstacle_distance_;

    std::string vision_target_name_;
    double assumed_depth_;
    double camera_fov_rad_;
    std::string image_topic_;

    static bool isSupportSurface(const std::string &name)
    {
        const std::string lower = toLower(name);
        return lower.find("table") != std::string::npos ||
               lower.find("shelf") != std::string::npos ||
               lower.find("counter") != std::string::npos ||
               lower.find("desk") != std::string::npos;
    }

    static std::string toLower(const std::string &s)
    {
        std::string out = s;
        std::transform(out.begin(), out.end(), out.begin(), ::tolower);
        return out;
    }

    static std::string normalizeClass(const std::string &name)
    {
        if (name.empty())
        {
            return name;
        }
        std::string norm = name;
        norm[0] = std::toupper(norm[0]);
        return norm;
    }

    bool updateCallback(world_percept_assig4::UpdateObjectList::Request &req,
                        world_percept_assig4::UpdateObjectList::Response &res)
    {
        processObservation(req.object_name, req.object_pose);
        res.confirmation = true;
        return true;
    }

    bool sceneCallback(world_percept_assig4::GetSceneObjectList::Request &req,
                       world_percept_assig4::GetSceneObjectList::Response &res)
    {
        geometry_msgs::Twist zero_twist;

        if (req.object_name == "all")
        {
            res.objects.name.clear();
            res.objects.pose.clear();
            res.objects.twist.clear();

            for (const auto &kv : objects_)
            {
                res.objects.name.push_back(kv.first);
                res.objects.pose.push_back(kv.second);
                res.objects.twist.push_back(zero_twist);
            }

            res.obj_found = true;
            return true;
        }

        auto it = objects_.find(req.object_name);
        if (it != objects_.end())
        {
            res.objects.name = {it->first};
            res.objects.pose = {it->second};
            res.objects.twist = {zero_twist};
            res.obj_found = true;
            return true;
        }

        // Use learned model to suggest a likely pose for unseen objects
        const std::string class_name = normalizeClass(req.object_name);
        double decision_confidence = 0.0;
        std::string best_surface = queryPreferredSurface(class_name, decision_confidence);
        if (best_surface.empty())
        {
            best_surface = predictSurface(class_name);
        }
        auto surface_it = supports_.find(best_surface);

        if (surface_it != supports_.end())
        {
            geometry_msgs::Pose predicted = surface_it->second;
            predicted.position.z += 0.05; // bias above surface

            res.objects.name = {req.object_name};
            res.objects.pose = {predicted};
            res.objects.twist = {zero_twist};
            res.obj_found = true;
            res.message = "Pose predicted from learned priors on surface: " + best_surface;
        }
        else
        {
            res.obj_found = false;
            res.message = "Object not observed and no surface prior available";
        }

        return true;
    }

    bool reportCallback(std_srvs::Trigger::Request &,
                        std_srvs::Trigger::Response &res)
    {
        std::string payload = buildConfusionJson();
        std_msgs::String msg;
        msg.data = payload;
        metrics_pub_.publish(msg);
        res.success = true;
        res.message = payload;
        return true;
    }

    void processObservation(const std::string &name, const geometry_msgs::Pose &pose)
    {
        objects_[name] = pose;

        if (isSupportSurface(name))
        {
            supports_[name] = pose;
            support_labels_.insert(name);
        }

        const std::string class_name = normalizeClass(name);
        const std::string actual_support = inferSupportLabel(name, pose);

        const std::string predicted_support = predictSurface(class_name);
        updateConfusion(predicted_support.empty() ? "untrained" : predicted_support, actual_support);

        updateStatistics(class_name, actual_support);
        assertKnowledge(class_name);
        assertObservationProlog(class_name, actual_support);

        // Publish confusion/state snapshot so external evaluators (e.g., sklearn) can compute metrics.
        std_msgs::String metrics;
        metrics.data = buildConfusionJson();
        metrics_pub_.publish(metrics);

        if (queryNeedsConfirmation(class_name))
        {
            ROS_WARN_STREAM_THROTTLE(5.0, "Knowledge for " << class_name << " needs confirmation between surfaces");
        }

        ROS_INFO_STREAM_THROTTLE(1.0, "Observed " << name << " on " << actual_support
                                                    << " (predicted: " << predicted_support << ")");
    }

    std::string inferSupportLabel(const std::string &obj_name, const geometry_msgs::Pose &pose) const
    {
        if (isSupportSurface(obj_name))
        {
            return obj_name;
        }

        double best_dist = std::numeric_limits<double>::infinity();
        std::string best_surface = "floor";

        for (const auto &kv : supports_)
        {
            const geometry_msgs::Pose &surf_pose = kv.second;
            const double dx = surf_pose.position.x - pose.position.x;
            const double dy = surf_pose.position.y - pose.position.y;
            const double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < best_dist)
            {
                best_dist = dist;
                best_surface = kv.first;
            }
        }

        return best_surface;
    }

    void updateStatistics(const std::string &class_name, const std::string &surface)
    {
        Stats &stats = learned_stats_[class_name];
        stats.surface_counts[surface] += 1;
        stats.total += 1;
        support_labels_.insert(surface);
    }

    void updateConfusion(const std::string &predicted, const std::string &actual)
    {
        confusion_[actual][predicted] += 1;
    }

    std::string predictSurface(const std::string &class_name) const
    {
        auto it = learned_stats_.find(class_name);
        if (it == learned_stats_.end() || it->second.total == 0 || support_labels_.empty())
        {
            return "";
        }

        const int surface_count = static_cast<int>(support_labels_.size());
        double best_score = -1.0;
        std::string best_surface;

        for (const auto &label : support_labels_)
        {
            const int count = it->second.surface_counts.count(label) ? it->second.surface_counts.at(label) : 0;
            const double prob = static_cast<double>(count + 1) / static_cast<double>(it->second.total + surface_count);
            if (prob > best_score)
            {
                best_score = prob;
                best_surface = label;
            }
        }

        return best_surface;
    }

    void assertKnowledge(const std::string &class_name)
    {
        if (asserted_classes_.count(class_name))
        {
            return;
        }

        world_percept_assig4::UpdateObjectList srv;
        srv.request.object_name = class_name;
        if (!reasoning_client_.exists())
        {
            reasoning_client_.waitForExistence(ros::Duration(0.5));
        }
        if (!reasoning_client_.call(srv))
        {
            ROS_WARN_STREAM_THROTTLE(2.0, "Could not assert knowledge for " << class_name);
            return;
        }

        if (srv.response.confirmation)
        {
            asserted_classes_.insert(class_name);
        }
    }

    std::string buildConfusionJson() const
    {
        std::ostringstream oss;
        oss << "{";
        bool first_actual = true;
        for (const auto &act_pair : confusion_)
        {
            if (!first_actual) oss << ",";
            first_actual = false;
            oss << "\"" << act_pair.first << "\":{";
            bool first_pred = true;
            for (const auto &pred_pair : act_pair.second)
            {
                if (!first_pred) oss << ",";
                first_pred = false;
                oss << "\"" << pred_pair.first << "\":" << pred_pair.second;
            }
            oss << "}";
        }
        oss << "}";
        return oss.str();
    }

    void assertObservationProlog(const std::string &class_name, const std::string &surface)
    {
        if (!prolog_ready_)
        {
            return;
        }

        std::ostringstream query;
        query << "observe_instance('" << class_name << "','" << surface << "',1.0).";

        try
        {
            pl_.query(query.str());
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM_THROTTLE(2.0, "Failed to assert observation in Prolog: " << e.what());
        }
    }

    std::string queryPreferredSurface(const std::string &class_name, double &confidence)
    {
        confidence = 0.0;
        if (!prolog_ready_)
        {
            return "";
        }

        best_surface_buffer_.clear();
        std::ostringstream query;
        query << "search_target('" << class_name << "',Surface,Confidence).";
        try
        {
            PrologQuery bdgs = pl_.query(query.str());
            for (auto &solution : bdgs)
            {
                for (auto &pair : solution)
                {
                    if (pair.first == "Surface")
                    {
                        best_surface_buffer_ = pair.second.toString();
                    }
                    else if (pair.first == "Confidence")
                    {
                        confidence = std::atof(pair.second.toString().c_str());
                    }
                }
            }
            bdgs.finish();
            return best_surface_buffer_;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM_THROTTLE(2.0, "Prolog search_target failed: " << e.what());
            return "";
        }
    }

    bool queryNeedsConfirmation(const std::string &class_name)
    {
        if (!prolog_ready_)
        {
            return false;
        }

        std::ostringstream query;
        query << "needs_confirmation('" << class_name << "').";
        try
        {
            PrologQuery bdgs = pl_.query(query.str());
            bool res = bdgs.begin() != bdgs.end();
            bdgs.finish();
            return res;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM_THROTTLE(2.0, "Prolog needs_confirmation failed: " << e.what());
            return false;
        }
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        double min_range = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
            if (std::abs(angle) > 0.35)
            {
                continue;
            }
            const double r = msg->ranges[i];
            if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max)
            {
                min_range = std::min(min_range, r);
            }
        }

        latest_obstacle_distance_ = min_range;
        std_msgs::Float32 out;
        out.data = std::isfinite(min_range) ? static_cast<float>(min_range) : msg->range_max;
        obstacle_pub_.publish(out);

        if (std::isfinite(min_range) && min_range < obstacle_threshold_)
        {
            geometry_msgs::PoseStamped obstacle_ps;
            obstacle_ps.header = msg->header;
            obstacle_ps.pose.orientation.w = 1.0;
            obstacle_ps.pose.position.x = min_range;
            obstacle_ps.pose.position.y = 0.0;
            obstacle_ps.pose.position.z = 0.0;

            try
            {
                geometry_msgs::PoseStamped world_pose = tf_buffer_.transform(obstacle_ps, "world", ros::Duration(0.05));
                processObservation("obstacle_front", world_pose.pose);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_DEBUG_STREAM_THROTTLE(2.0, "Could not transform obstacle to world: " << ex.what());
            }
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        }
        catch (const cv_bridge::Exception &e)
        {
            ROS_WARN_STREAM_THROTTLE(2.0, "cv_bridge failed: " << e.what());
            return;
        }

        cv::Mat hsv;
        cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2);
        cv::bitwise_or(mask1, mask2, mask);

        const double ratio = static_cast<double>(cv::countNonZero(mask)) / static_cast<double>(mask.total());
        if (ratio < 0.01)
        {
            return;
        }

        const cv::Moments m = cv::moments(mask, true);
        if (m.m00 <= 1e-3)
        {
            return;
        }

        const double cx = m.m10 / m.m00;
        const double center_offset = (cx - static_cast<double>(mask.cols) / 2.0) / (static_cast<double>(mask.cols) / 2.0);
        const double bearing = center_offset * (camera_fov_rad_ / 2.0);

        geometry_msgs::PoseStamped cam_pose;
        cam_pose.header = msg->header;
        cam_pose.pose.orientation.w = 1.0;
        cam_pose.pose.position.x = assumed_depth_ * std::cos(bearing);
        cam_pose.pose.position.y = assumed_depth_ * std::sin(bearing);
        cam_pose.pose.position.z = 0.8; // approximate camera height

        geometry_msgs::PoseStamped world_pose;
        try
        {
            world_pose = tf_buffer_.transform(cam_pose, "world", ros::Duration(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_STREAM_THROTTLE(2.0, "Transform camera->world failed: " << ex.what());
            return;
        }

        processObservation(vision_target_name_, world_pose.pose);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "learning_node");
    ros::NodeHandle nh;
    LearningNode node(nh);
    ros::spin();
    return 0;
}
