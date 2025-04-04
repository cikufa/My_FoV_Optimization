#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>
#include <vector>
#include <fstream> 
#include <sstream>

class PoseSE3{
public:
    PoseSE3(Eigen::Vector3f p,Eigen::Matrix3f rot){
        this->position=p;
        this->rotation=rot;
    }
    PoseSE3(){  
        this->position<<0,0,0;//zero
        this->rotation<<1,0,0,0,1,0,0,0,1; //identity
    }


    Eigen::Matrix3f get_rotation(){
        return this->rotation;

    }
    void set_rotation(Eigen::Matrix3f rot){
        this->rotation=rot;

    }

    Eigen::Vector3f get_position(){
        return this->position;

    }

    void set_position(Eigen::Vector3f p){
        this->position=p;
    }

private:
    Eigen::Vector3f position;
    Eigen::Matrix3f rotation;

};




class OdomToVector3Converter {
public:

    std::vector<std::string> split_string(std::string s,std::string delimiter){
        size_t last = 0;
        size_t next = 0;
        std::vector<std::string> splitted_string;
        //std::string delimiter=delimiter;
        while ((next = s.find(delimiter, last)) != std::string::npos) {   
            // std::cout << s.substr(last, next-last) << std::endl;
            splitted_string.push_back(s.substr(last, next-last));
            last = next + 1; 
            } 
        splitted_string.push_back(s.substr(last));
        // std::cout << s.substr(last) << std::endl;
        return splitted_string;
    }

    OdomToVector3Converter(std::string data_filename,int downsample_factor,std::string delimiter) {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle("~");

        // Subscribe to the odometry topic
        odom_sub_ = nh_.subscribe("/localization", 1, &OdomToVector3Converter::odomCallback, this);

        // Publish the 3-vector on a new topic
        vector3_pub_ = nh_.advertise<geometry_msgs::Vector3>("/cam_motor_control", 1);
        // load trajectory file

        ImportFromTrajectoryFile(data_filename,downsample_factor,delimiter);

    }


    void ImportFromTrajectoryFile(std::string data_filename,int downsample_factor,std::string delimiter){

        std::vector<PoseSE3> trajectory;

        //this->filename=data_filename;
        this->data_file.open (data_filename);

        if (!this->data_file.is_open()) {
            std::cerr << "Failed to open the file for reading." << std::endl;
        } else {
            std::cout << "File opened successfully for reading." << std::endl;
            // You can read from the file here.
        }
        std::string line;
        int count=0;
        while (getline (this->data_file, line)) {

            if (count%downsample_factor!=0) {
                count+=1;
                continue;
            }
            std::cout<<"line "<<line<<std::endl;
            std::vector<std::string> splitted_line=this->split_string(line,delimiter);
            std::cout<<"splitted line length"<<splitted_line.size()<<std::endl;
            for(std::string token:splitted_line)
                std::cout<<"token "<<token<<std::endl;
            float pos_x=std::stof(splitted_line[0]);
            float pos_y=std::stof(splitted_line[1]);
            float pos_z=std::stof(splitted_line[2]);
            float dir_x=std::stof(splitted_line[3]);
            float dir_y=std::stof(splitted_line[4]);
            float dir_z=std::stof(splitted_line[5]);

            Eigen::Vector3f position;
            Eigen::Vector3f direction;
            position<<pos_x,pos_y,pos_z;
            direction<<dir_x,dir_y,dir_z;
            this->traj_positions.push_back(position);
            this->traj_directions.push_back(direction);
            count+=1;
        }

        this->data_file.close();
        std::cout << "import trajectory successfully." << std::endl;
        std::cout<< "position length" <<this->traj_positions.size()<<std::endl;
        std::cout<< "dir length" <<this->traj_directions.size()<<std::endl;
        //return trajectory;
    }




    Eigen::Vector3f lookup_best_FOV(Eigen::Vector3f current_position){
        float min_norm=10000000;
        Eigen::Vector3f best_dir;
        for(int i=0;i<this->traj_positions.size();i++){
            Eigen::Vector3f position=this->traj_positions[i];
            Eigen::Vector3f distance=position-current_position;
            if (distance.norm()<min_norm){
                min_norm=distance.norm();
                best_dir=this->traj_directions[i];
            }
        }

        return best_dir;
    }



    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        // Extract linear position from the odometry message
        double x = odom_msg->pose.pose.position.x;
        double y = odom_msg->pose.pose.position.y;
        double z = odom_msg->pose.pose.position.z;

        double q_w=odom_msg->pose.pose.orientation.w;
        double q_x=odom_msg->pose.pose.orientation.x;
        double q_y=odom_msg->pose.pose.orientation.y;
        double q_z=odom_msg->pose.pose.orientation.z;
        Eigen::Vector3f current_position;
        current_position<<x,y,z;
        Eigen::Vector3f best_dir=lookup_best_FOV(current_position);
        float best_world_yaw=atan2(best_dir[1],best_dir[0])*180.0/M_PI;


        Eigen::Quaternionf quaternion(q_w, q_x, q_y, q_z);  // Example quaternion (w, x, y, z)
        // Convert quaternion to rotation matrix
        Eigen::Matrix3f rotation_matrix = quaternion.toRotationMatrix();
        Eigen::Vector3f v;
        v<<1,0,0;
        v=rotation_matrix*v;
        float current_yaw=atan2(v[1],v[0])*180.0/M_PI;

        float yaw_control=best_world_yaw-current_yaw;

        // Create a 3-vector message
        geometry_msgs::Vector3 vector3_msg;
        vector3_msg.x = yaw_control;
        vector3_msg.y = 0.0;
        vector3_msg.z = 0.0;
        // Publish the 3-vector message
        vector3_pub_.publish(vector3_msg);
    }

private:


    std::vector<Eigen::Vector3f> traj_positions;
    std::vector<Eigen::Vector3f> traj_directions;

    std::ifstream data_file;
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher vector3_pub_;
};

int main(int argc, char** argv) {
    // Initialize the ROS node

    // if(argc!=3){

    //     std::cout<<"Usage: add argument <file name> <downsample_factor>"<<std::endl;
    //     return 0;
    // }
    ros::init(argc, argv, "odom_to_vector3_converter");

     
    ros::NodeHandle nh;

    std::string file_name;
    nh.getParam("/controller/file_name", file_name);  
    std::cout<<"file_name is "<<file_name<<std::endl;
    int sub_sampling_interval;
    nh.getParam("/controller/sub_sample_interval", sub_sampling_interval);  

    
    // Create an instance of the converter class
    OdomToVector3Converter converter(file_name,sub_sampling_interval,",");

    // Spin to process callbacks
    ros::spin();

    return 0;
}