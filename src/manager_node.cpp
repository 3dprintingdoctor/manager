#include "manager_node.h"
#include <pointclouds.h>

bool srv_obj_rec = true;
bool srv_points_grasp = false;

double sol_int0 = 0.0;
double total_s = 100;

float total_time0 = 0.;

ros::Time t_init;
ros::Time t_curr;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manager_node");

    ros::NodeHandle nh_;

    //std::string path = ros::package::getPath("manager") + "/txt/task.txt";

    ros::service::waitForService("objects_recognition");
    ros::ServiceClient client = nh_.serviceClient<manager_rec_msgs::objectsRecognition>  ("objects_recognition");
    manager_rec_msgs::objectsRecognition srv;

    srv.request.userInputsFile  = "/home/users/aleix.ripoll/kinnrec_w/src/manager/txt/task.txt";
    ros::Rate loop_rate(10);
    double x, y, z, ax, ay, az, theta;

    double x_real = 0.0;
    double y_real = 0.0;
    double z_real = 0.165;
    double ax_real = 0.0;
    double ay_real = 1.0;
    double az_real = 0.0;
    double theta_real = M_PI_2;

    double De_x;
    double De_y;
    double De_z;
    double De_ax;
    double De_ay;
    double De_az;
    double De_theta;

    std::ofstream myfile;
    myfile.open ("/home/users/aleix.ripoll/tests_object_recognition/x_estimated.txt");
    std::ofstream myfile2;
    myfile2.open ("/home/users/aleix.ripoll/tests_object_recognition/x_error.txt");
    std::ofstream myfile3;
    myfile3.open ("/home/users/aleix.ripoll/tests_object_recognition/y_estimated.txt");
    std::ofstream myfile4;
    myfile4.open ("/home/users/aleix.ripoll/tests_object_recognition/y_error.txt");
    std::ofstream myfile5;
    myfile5.open ("/home/users/aleix.ripoll/tests_object_recognition/z_estimated.txt");
    std::ofstream myfile6;
    myfile6.open ("/home/users/aleix.ripoll/tests_object_recognition/z_error.txt");
    std::ofstream myfile7;
    myfile7.open ("/home/users/aleix.ripoll/tests_object_recognition/ax_estimated.txt");
    std::ofstream myfile8;
    myfile8.open ("/home/users/aleix.ripoll/tests_object_recognition/ax_error.txt");
    std::ofstream myfile9;
    myfile9.open ("/home/users/aleix.ripoll/tests_object_recognition/ay_estimated.txt");
    std::ofstream myfile10;
    myfile10.open ("/home/users/aleix.ripoll/tests_object_recognition/ay_error.txt");
    std::ofstream myfile11;
    myfile11.open ("/home/users/aleix.ripoll/tests_object_recognition/az_estimated.txt");
    std::ofstream myfile12;
    myfile12.open ("/home/users/aleix.ripoll/tests_object_recognition/az_error.txt");
    std::ofstream myfile13;
    myfile13.open ("/home/users/aleix.ripoll/tests_object_recognition/theta_estimated.txt");
    std::ofstream myfile14;
    myfile14.open ("/home/users/aleix.ripoll/tests_object_recognition/theta_error.txt");

    while (ros::ok()) {

        if(srv_obj_rec)
        {

            t_init = ros::Time::now();
            if (client.call(srv))
            {
                if(!srv.response.objectsList.vector_objects.empty())
                {
                    t_curr = ros::Time::now();
                    ros::Duration dt= t_curr - t_init;
                    total_time0 += dt.toSec();
                    std::string name1=srv.response.objectsList.vector_name;
                    for(unsigned int c=0; c<srv.response.objectsList.vector_len; ++c){
                         x = srv.response.objectsList.vector_objects.at(c).TemplatePose.x;
                         y = srv.response.objectsList.vector_objects.at(c).TemplatePose.y;
                         z = srv.response.objectsList.vector_objects.at(c).TemplatePose.z;
                         ax = srv.response.objectsList.vector_objects.at(c).TemplatePose.ax;
                         ay = srv.response.objectsList.vector_objects.at(c).TemplatePose.ay;
                         az = srv.response.objectsList.vector_objects.at(c).TemplatePose.az;
                         theta = srv.response.objectsList.vector_objects.at(c).TemplatePose.theta;

                    }
//                    std::cout<<"Pose object "<<name1<<" : x "<<x<<" y "<<y<<" z "<<z<<std::endl
//                            <<" angle "<<theta<<" ax "<<ax<<" ay "<<ay<<" az "<<az<<std::endl;
//                    //srv_obj_rec = false;
//                    srv_points_grasp = false;
                }
                else{
                    std::cout<<" Objects List is empty."<<std::endl;
                }
            }
            else
            {
              ROS_ERROR("Failed to call service objects_recognition");
              return 1;
            }  

            De_x = x - x_real;
            De_y = y - y_real;
            De_z = z - z_real;
            De_ax = ax - ax_real;
            De_ay = ay - ay_real;
            De_az = az - az_real;
            De_theta = theta - theta_real;

            myfile << x <<",";
            myfile2 << De_x <<",";
            myfile3 << y <<",";
            myfile4 << De_y <<",";
            myfile5 << z <<",";
            myfile6 << De_z <<",";
            myfile7 << ax <<",";
            myfile8 << De_ax <<",";
            myfile9 << ay <<",";
            myfile10 << De_ay <<",";
            myfile11 << az <<",";
            myfile12 << De_az <<",";
            myfile13 << theta <<",";
            myfile14 << De_theta <<",";

            ++sol_int0;


            if(sol_int0 == total_s){
                srv_obj_rec = false;
                std::cout<<" -> Average time (s): "<<(total_time0/total_s)<<std::endl;
                myfile.close();
                myfile2.close();
                myfile3.close();
                myfile4.close();
                myfile5.close();
                myfile6.close();
                myfile7.close();
                myfile8.close();
                myfile9.close();
                myfile10.close();
                myfile11.close();
                myfile12.close();
                myfile13.close();
                myfile14.close();
            }

        }
        if(srv_points_grasp){
            // Grasp module Abiud: objects pose AngleAxis -> |G| -> vector of points(pose)

            //inputs:
            pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile<pcl::PointXYZ>("/home/users/aleix.ripoll/kinnrec_w/src/camera_sens/models_dataset/Rugby.pcd", *object);

            std::string name = "Rugby.";
            std::vector<float> objectPose(7);
            objectPose.at(0)=x;
            objectPose.at(1)=y;
            objectPose.at(2)=z;
            objectPose.at(3)=ax;
            objectPose.at(4)=ay;
            objectPose.at(5)=az;
            objectPose.at(6)=theta;
            bool sol = true;

            //Call lib


            GA2H *Grasp;
            Grasp = new GA2H();

            Grasp->computeContactPoints(object,name,objectPose,sol);
            srv_points_grasp = false;
            return 0;
        }

        //S'ha de determinar la qfinal, ja que el Kautham la necessita per executar els planners(PRM, RRT, human like)

       ros::spinOnce();
       loop_rate.sleep();
    }
    return 0;
}


