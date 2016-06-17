#include "manager_node.h"

/*bool found(manager::objectsRecognition::Request  &req, manager::objectsRecognition::Response &res)
{

    const char* Filename = req.userInputsFile.c_str();


    std::ifstream myfile(Filename);


    if(myfile.is_open())
    {
        res.read = true;
        myfile.close();
    }
    else
    {
        std::cout<<"Unable to open file. Shutdown the program and check it."<<std::endl;
        res.read = false;
    }
  return true;
}*/


int main(int argc, char **argv)
{

  ros::init(argc, argv, "server_node");
  ros::NodeHandle n;
  
  //ros::ServiceServer service = n.advertiseService("objects_recognition", found);
  ROS_INFO("Ready to recognize objects.");
  ros::spin();
  return 0;
}
