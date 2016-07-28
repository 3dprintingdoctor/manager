#include "manager_node.h"
#include <pointclouds.h>

bool srv_obj_rec = true;
bool srv_points_grasp = false;
bool srv_ikn = false;

//const int maxiter = 100;
//const double err = 1e-4;
//Eigen::Vector3d position;
//Eigen::Vector3d orientation;
//IKN::SolverType solverName = IKN::KDL_PINV;

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

   // ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states_2", 1);
    //sensor_msgs::JointState joint_state;
   // IKN::Robot robot0("/home/users/aleix.ripoll/kinnrec_w/src/manager/urdf/UR5.urdf");
    //Eigen::VectorXd q_init(6);
    //Eigen::Matrix4f H_pp;


    while (ros::ok()) {

        if(srv_obj_rec)
        {
            if (client.call(srv))
            {
                if(!srv.response.objectsList.vector_objects.empty())
                {

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

                    std::cout<<"Pose object "<<name1<<" : x "<<x<<" y "<<y<<" z "<<z<<std::endl
                            <<" angle "<<theta<<" ax "<<ax<<" ay "<<ay<<" az "<<az<<std::endl;
                    srv_obj_rec = false;
                    srv_points_grasp = false;
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
            //srv_ikn = true;
            return 0;
        }



        //S'ha de determinar la qfinal, ja que el Kautham la necessita per executar els planners(PRM, RRT, human like)
        /*if(srv_ikn){
            std::cout<<"position: "<<position(0)<<" "<<position(1)<<" "<<position(2)<<std::endl;
            std::cout<<"orientation: "<<orientation(0)<<" "<<orientation(1)<<" "<<orientation(2)<<std::endl;


            Eigen::VectorXd Q;
            double curr_error;
            q_init(0)=0.0;
            q_init(1)=0.0;
            q_init(2)=0.0;
            q_init(3)=0.0;
            q_init(4)=0.0;
            q_init(5)=0.0;

            Q = robot0.inverse_kinematics(q_init, position, orientation, solverName, maxiter, err, curr_error);

            if(solverName == 0){
                std::cout<< "Solver: KDL PINV"<<std::endl;
            }
            else if(solverName == 1){
                std::cout<< "Solver: OWN JACOBIAN PINV"<<std::endl;
            }
            else if (solverName == 2){
                std::cout<< "Solver: OWN JACOBIAN TRANSPOSE"<<std::endl;
            }
            else if(solverName == 3){
                std::cout<< "Solver: OWN JACOBIAN DLS"<<std::endl;
            }

            std::cout << "Q : "<<std::endl
                      <<Q<<std::endl;
            std::cout << "Current Error : "<<curr_error<<std::endl;

            if(Q.size()>0)
            {
                joint_state.header.stamp = ros::Time::now();

                joint_state.name.resize(robot0.JointsName.size()); //sorted by name
                joint_state.name[0] = robot0.JointsName.at(2); //"ShoulderPanJoint"
                joint_state.name[1] = robot0.JointsName.at(1); //"ShoulderLiftJoint"
                joint_state.name[2] = robot0.JointsName.at(0); //"ElbowJoint"
                joint_state.name[3] = robot0.JointsName.at(3); //"Wrist1Joint";
                joint_state.name[4] = robot0.JointsName.at(4); //"Wrist2Joint";
                joint_state.name[5] = robot0.JointsName.at(5); //"Wrist3Joint";


                joint_state.position.resize(Q.size());
                for(unsigned int k=0; k<Q.size(); ++k){
                    joint_state.position[k] = Q(k);
                }

                joint_pub.publish(joint_state);
            }

        }*/

       ros::spinOnce();
       loop_rate.sleep();
    }
    return 0;
}


