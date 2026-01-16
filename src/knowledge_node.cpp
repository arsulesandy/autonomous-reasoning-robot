#include <ros/ros.h>
#include <fstream>
#include <string>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <final_project/LoadKnowledge.h>

class Knowledge
{
public:
    explicit Knowledge(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if (pl_.waitForServer())
        {
            pl_ = PrologClient("/rosprolog", true);
        }

        srv_load_knowledge_name_ = "load_knowledge";
        load_knowledge_srv_ = nh.advertiseService(srv_load_knowledge_name_, &Knowledge::callback_load_knowledge, this);
    }

    void setQueryFile(const std::string &fileName_Q)
    {
        query_file_.close();
        query_file_.clear();
        query_file_.open(fileName_Q);

        if (!query_file_.is_open())
        {
            ROS_ERROR_STREAM("File not found and exit the function");
            return;
        }

        ROS_INFO_STREAM("Loading queries from: " << fileName_Q);
    }

private:
    PrologClient pl_;
    std::string srv_load_knowledge_name_;
    ros::ServiceServer load_knowledge_srv_;
    std::ifstream query_file_;

    bool callback_load_knowledge(final_project::LoadKnowledge::Request &req,
                                 final_project::LoadKnowledge::Response &res)
    {
        if (req.start != 1)
        {
            res.confirm = false;
            ROS_WARN_STREAM("LoadKnowledge requested with start != 1, ignoring");
            return true;
        }

        if (!query_file_.is_open())
        {
            ROS_ERROR_STREAM("Query file is not open");
            res.confirm = false;
            return true;
        }

        loadQueries();
        res.confirm = true;
        return true;
    }

    void loadQueries()
    {
        if (!query_file_.is_open())
        {
            ROS_ERROR_STREAM("Query file is not open");
            return;
        }

        std::string line;
        while (std::getline(query_file_, line))
        {
            if (line.empty())
            {
                continue;
            }

            pl_.query(line);
        }

        // Allow re-loading on subsequent requests
        query_file_.clear();
        query_file_.seekg(0);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "knowledge_node");

    if (argc < 2)
    {
        ROS_ERROR_STREAM("Missing base path for query file. Usage: rosrun final_project knowledge_node <path>");
        return 1;
    }

    ros::NodeHandle nh;
    Knowledge knowledge(nh);

    std::string query_file_param;
    nh.param<std::string>("read_prolog_queries/saved_query_file", query_file_param, std::string());

    if (query_file_param.empty())
    {
        ROS_ERROR_STREAM("Parameter read_prolog_queries/saved_query_file not set");
        return 1;
    }

    std::string base_path = argv[1];
    if(!base_path.empty() && base_path.back() != '/')
    {
        base_path.push_back('/');
    }
    std::string file_path = base_path + query_file_param;
    knowledge.setQueryFile(file_path);

    ros::spin();
    return 0;
}
