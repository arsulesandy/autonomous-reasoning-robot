#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <sstream>
#include <iostream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <final_project/UpdateObjectList.h>

using namespace std;

class Reasoner
{
private:
    PrologClient pl_;
    int ID_;

    std::string srv_assert_knowledge_name_;
    ros::ServiceServer assert_knowledge_srv_;                            // Advertise service to assert knowledge in the ontology

    //Variable to save our preference to save or not the asserted queries
    bool m_query_flag_save;
    std::ofstream out_queries_file_;
public:

    Reasoner(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        ID_=0; //Global variable to include in the asserted instances

        srv_assert_knowledge_name_ = "assert_knowledge";
        assert_knowledge_srv_ = nh.advertiseService(srv_assert_knowledge_name_, &Reasoner::srv_assert_callback, this);

        this->m_query_flag_save=false;
    };

    ~Reasoner(){
        out_queries_file_.close();
    };

   //TODO A04.T02: This function should open a file if the path of the file is found, otherwise it should print out that the file or directory were not found (1 pt)
    void setOutQueriesFile(string QueryfileName)
    {
        out_queries_file_.close();
        out_queries_file_.clear();
        out_queries_file_.open(QueryfileName, std::ios::out | std::ios::app);

        if(out_queries_file_.is_open())
        {
            ROS_INFO_STREAM("query_file: "<< QueryfileName);
            this->m_query_flag_save=true; //This means that I want to save the queries in a file
        }
        else
        {
            ROS_ERROR_STREAM("File not found and exit the function");
            this->m_query_flag_save=false;
        }
    }

private:


     /**
     * @brief Callback function for the service that adds objects to the map_objects list
     *
     * @param Request requested object to be added to the knowledge base
     * @param Respose response from the service when the object has been asserted (true/false)
     */
    bool srv_assert_callback(final_project::UpdateObjectList::Request &req,
                             final_project::UpdateObjectList::Response &res)
    {
        ROS_INFO_STREAM("Got new object: " << req.object_name);
        std::string object;

        //TODO: Modify this callback function to first verify that the seen object has a class, then the seen object can be asserted into the knowledge base. The response of this function is true if the assertion of knowledge is succesful.

        object=req.object_name;
        res.confirmation = false;

        bool has_class = getClass(object);

        if(has_class)
        {
            res.confirmation = assertKnowledge(object);
        }
        else
        {
            ROS_WARN_STREAM("Could not verify or create class for: "<<object);
        }

        return res.confirmation;
    }


    bool getClass(std::string className)
    {

        // TODO: Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils"
        std::string query= "get_class('"+className+"').";

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);

        bool res = false;
        for (auto &it : bdgs)
        {
            res = true;
            ROS_INFO_STREAM("A new class was created in the ontology");
            break;
        }

        bdgs.finish();

        if(m_query_flag_save && out_queries_file_.is_open())
        {
            out_queries_file_<<query<<std::endl;
        }

        return res;
    }

    bool assertKnowledge(std::string className)
    {
        std::string instanceName;

        // TODO: Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils"
        std::stringstream ss;
        ss<<"create_instance_from_class('"<<className<<"',"<<ID_<<",Instance).";
        std::string query= ss.str();

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);

        bool got_instance=false;

        for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
        {
            for (auto val : *it)
            {
                //TODO: Retrive the value from Prolog
                if(val.first == "Instance")
                {
                    instanceName = val.second.toString();
                }
                ROS_WARN_STREAM("new instance in knowledge base: "<<instanceName);
                got_instance=true;
            }

        }

        bdgs.finish();

        if(got_instance && m_query_flag_save && out_queries_file_.is_open())
        {
            out_queries_file_<<query<<std::endl;
        }

        if(got_instance)
        {
            ID_++;
        }

        return got_instance;
    }

}; //class Reasoner

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "reasoning_node");

  ros::NodeHandle nh;

  if(argc < 2)
  {
      ROS_ERROR_STREAM("Missing base path for query file. Usage: rosrun final_project reasoning_node <path>");
      return 1;
  }

  Reasoner myReasoner(nh);

   //+ Information about the path for the file that will save the queries
  std::string saveFilePath;
  saveFilePath = argv[1]; // This means that the node expects a path value as input. This means that we need to run this node as follows: rosrun world_percept

  if(!saveFilePath.empty() && saveFilePath.back()!='/')
  {
      saveFilePath.push_back('/');
  }


   bool saveQueries_flag=false; //variable that will receive the value from the yaml file

   //TODO A04.T02: Retrieve the variable from the yaml file and save it in the new variable "saveQueries_flag" (0.25 pts)
   if(!nh.getParam("read_prolog_queries/save_flag", saveQueries_flag))
   {
       ROS_WARN_STREAM("Parameter read_prolog_queries/save_flag not found, defaulting to false");
   }
   std::string savedQueryFile;

   if(saveQueries_flag)
    { //If the flag is true, then we will configure the file to save the asserted queries
         //TODO A04.T02: Include the code to load the rosparam from a yaml file (0.75 pt)
         // First define a new string variable "savedQueryFile"
         // Then load the yaml parameter in the new variable
         if(!nh.getParam("read_prolog_queries/saved_query_file", savedQueryFile))
         {
             ROS_ERROR_STREAM("Parameter read_prolog_queries/saved_query_file not found");
             saveQueries_flag=false;
         }

        if(saveQueries_flag)
        {
            //This node now needs a path as input when we run it. This path is addedd to the obtain variable from the yaml file, as follows
            savedQueryFile= saveFilePath+savedQueryFile;
            ROS_INFO_STREAM("query_file: "<< savedQueryFile);

            //Now we call a new function which will create and open the new file
            myReasoner.setOutQueriesFile(savedQueryFile);
        }

    }

  ros::spin();


  return 0;
}
