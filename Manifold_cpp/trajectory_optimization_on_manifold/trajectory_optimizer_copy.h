#ifndef _TRAJECTORYOPMANIFOLD_


#define _TRAJECTORYOPMANIFOLD_


#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <time.h>
#include <vector>
#include <typeinfo>
#include <chrono>
#include <fstream>
#include <sstream>
#include <cloud_loader.h>
#include <iomanip>


template <typename T> void print(T x)
{


   std::cout<<x<<std::endl;
}


class myTrajectoryOptimizerOnManifold{
public:


   myTrajectoryOptimizerOnManifold(std::string output_initial_trajectory_filename,
       std::string output_initial_trajectory_filename_ue,
       std::string output_initial_trajectory_filename_twc,
       std::string input_pointcloud_filename,
       std::string output_pointcloud_filename,
       bool use_direction, bool use_uncertainty,
       std::string input_direction_and_uncertainty_filename,
       std::string output_pointcloud_dir_filename,
       std::string input_trajectory_file,
       std::string output_trajectory_file,
       std::string output_trajectory_filename_ue,
       std::string output_trajectory_filename_twc){
      
       this->myinitialization(output_initial_trajectory_filename, output_initial_trajectory_filename_ue, output_initial_trajectory_filename_twc,
           input_pointcloud_filename,output_pointcloud_filename,use_direction,use_uncertainty,input_direction_and_uncertainty_filename,
           output_pointcloud_dir_filename,input_trajectory_file,output_trajectory_file, output_trajectory_filename_ue, output_trajectory_filename_twc);
   };


   void myinitialization(
   const std::string& output_initial_trajectory_filename,
   const std::string& output_initial_trajectory_filename_ue,  //gotta have xyz matched with stamped_twc, and  yaw along path
   const std::string& output_initial_trajectory_filename_twc,
   const std::string& input_pointcloud_filename,
   const std::string& output_pointcloud_filename,
   bool use_direction,
   bool use_uncertainty,
   //ignore
   const std::string& input_direction_and_uncertainty_filename,
   const std::string& output_pointcloud_dir_filename,  //dir means direction
  
   const std::string& input_trajectory_file_name,
   const std::string& output_trajectory_filename,
   const std::string& output_trajectory_filename_ue,
   const std::string& output_trajectory_filename_twc){
       //ignore
       this->use_uncertainty=use_uncertainty;
       this->use_direction=use_direction;
       this->montecarlopointsdirfile.open(output_pointcloud_dir_filename);

       this->output_initial_trajectory_filename=output_initial_trajectory_filename;
	   this->output_initial_trajectory_filename_ue = output_initial_trajectory_filename_ue;
       this->output_initial_trajectory_filename_twc = output_initial_trajectory_filename_twc;
    	this->initial_trajectory_file.open(this->output_initial_trajectory_filename);

	   this->output_trajectory_file.open(output_trajectory_filename);
       this->output_trajectory_filename_ue= output_trajectory_filename_ue;
       this->output_trajectory_filename_twc= output_trajectory_filename_twc;


       this->v<<1,1,1;
       this->v=this->v/this->v.norm();
       this->c<<1,0,0;
       this->c=this->c/this->c.norm();
       this->pcdfile.open(output_pointcloud_filename);
       this->trajectory= this->import_trajectory(input_trajectory_file_name," ", true);
       std::cout<<this->trajectory[1].get_position()<<std::endl;
       std::cout<<this->trajectory[1].get_rotation()<<std::endl;
       Eigen::Vector3f starting_position;
       // starting_position<<this->trajectory->position;
       // starting_position = this->trajectory[0].get_position();
      
       //NOTE: starting_rotation<<this->trajectory->rotation; //change to robot orientation along path yaw = arctan2(deltay/deltax)
       // Eigen::Matrix3f starting_rotation;
       // starting_rotation<<1,0,0,0,1,0,0,0,1;
       // starting_rotation = this->trajectory[0].get_rotation();


       this->loader.ImportFromXyzFile(input_pointcloud_filename,1,true,false," ");
       // if(this->use_direction||this->use_uncertainty){
       //      this->loader.ImportFromDirUncertaintyFile(input_direction_and_uncertainty_filename,1," ");
       // }
       //save pointcloud
       for (Eigen::Vector3f point:this->loader.get_pointcloud()){
           this->pcdfile<<point[0]<<","<<point[1]<<","<<point[2]<<std::endl;
       }
       this->pcdfile.close();
       std::cout<<"pcd entries written: "<<this->loader.get_pointcloud().size()<<std::endl;


       //save pointcloud_dir and uncertainty
       // std::vector<Eigen::Vector3f> pointcloud_dir_vector=this->loader.get_pointcloud_dir();
       // std::vector<float> points_uncertainty_vector=this->loader.get_pointcloud_uncertainty();
       // for(int i=0;i<pointcloud_dir_vector.size();i++){
       //   this->montecarlopointsdirfile<<pointcloud_dir_vector[i][0]<<","<<pointcloud_dir_vector[i][1]<<","<<pointcloud_dir_vector[i][2]<<","<<points_uncertainty_vector[i]<<std::endl;
       // }
       // this->montecarlopointsdirfile.close();


       this->valid_points.clear();


       for (size_t i=0;i<this->loader.get_pointcloud().size();i++){
           this->valid_points.push_back(i);
       }
   }


   // UE pose conversion based on unrealcv_bridge ue_conversions.py and esim_utils.hpp.
   double clampAxis(double angle_deg)
   {
       angle_deg = std::fmod(angle_deg, 360.0);
       if (angle_deg < 0.0) {
           angle_deg += 360.0;
       }
       return angle_deg;
   }

   double normalizeAxis(double angle_deg)
   {
       angle_deg = clampAxis(angle_deg);
       if (angle_deg > 180.0) {
           angle_deg -= 360.0;
       }
       return angle_deg;
   }

   void quaternionToEulerUnrealEngine(const Eigen::Quaterniond& q,
                                      double& yaw, double& pitch, double& roll)
   {
       const double X = q.x();
       const double Y = q.y();
       const double Z = q.z();
       const double W = q.w();

       const double SingularityTest = Z * X - W * Y;
       const double YawY = 2.0 * (W * Z + X * Y);
       const double YawX = (1.0 - 2.0 * (Y * Y + Z * Z));

       const double SINGULARITY_THRESHOLD = 0.4999995;
       const double RAD_TO_DEG = (180.0) / M_PI;

       if (SingularityTest < -SINGULARITY_THRESHOLD) {
           pitch = -90.0;
           yaw = std::atan2(YawY, YawX) * RAD_TO_DEG;
           roll = normalizeAxis(-yaw - (2.0 * std::atan2(X, W) * RAD_TO_DEG));
       } else if (SingularityTest > SINGULARITY_THRESHOLD) {
           pitch = 90.0;
           yaw = std::atan2(YawY, YawX) * RAD_TO_DEG;
           roll = normalizeAxis(yaw - (2.0 * std::atan2(X, W) * RAD_TO_DEG));
       } else {
           pitch = std::asin(2.0 * (SingularityTest)) * RAD_TO_DEG;
           yaw = std::atan2(YawY, YawX) * RAD_TO_DEG;
           roll = std::atan2(-2.0 * (W * X + Y * Z),
                             (1.0 - 2.0 * (X * X + Y * Y))) *
                  RAD_TO_DEG;
       }
   }

   Eigen::Matrix3d unrealEulerToRotationMatrix(const double pitch_deg,
                                               const double yaw_deg,
                                               const double roll_deg)
   {
       const double kDegToRad = M_PI / 180.0;
       const double roll_rad = -kDegToRad * roll_deg;
       const double pitch_rad = -kDegToRad * pitch_deg;
       const double yaw_rad = kDegToRad * yaw_deg;

       const Eigen::AngleAxisd rot_roll(roll_rad, Eigen::Vector3d::UnitX());
       const Eigen::AngleAxisd rot_pitch(pitch_rad, Eigen::Vector3d::UnitY());
       const Eigen::AngleAxisd rot_yaw(yaw_rad, Eigen::Vector3d::UnitZ());
       return (rot_yaw * rot_pitch * rot_roll).toRotationMatrix();
   }

   Eigen::Matrix4d getT_wue_w() const
   {
       Eigen::Matrix4d T_wue_w = Eigen::Matrix4d::Identity();
       T_wue_w(1, 1) = -1.0;
       return T_wue_w;
   }

   Eigen::Matrix4d getT_c_cue() const
   {
       Eigen::Matrix4d T_c_cue = Eigen::Matrix4d::Zero();
       T_c_cue(0, 1) = 1.0;
       T_c_cue(1, 2) = -1.0;
       T_c_cue(2, 0) = 1.0;
       T_c_cue(3, 3) = 1.0;
       return T_c_cue;
   }

   void TwcToUEPose(const Eigen::Matrix4f& Twc,
                    Eigen::Vector3f* posUE,
                    Eigen::Vector3f* eulerUE)
   {
       const Eigen::Matrix4d Twc_d = Twc.cast<double>();
       const Eigen::Matrix4d ue_Twc = getT_wue_w() * Twc_d * getT_c_cue();

       const Eigen::Matrix3d R = ue_Twc.block<3, 3>(0, 0);
       const Eigen::Quaterniond q(R);

       double yaw, pitch, roll;
       quaternionToEulerUnrealEngine(q, yaw, pitch, roll);

       (*posUE) = ue_Twc.block<3, 1>(0, 3).cast<float>();
       (*eulerUE) =
           Eigen::Vector3f(static_cast<float>(pitch),
                           static_cast<float>(yaw),
                           static_cast<float>(roll));
   }

   Eigen::Matrix4f UEPoseToTwc(const Eigen::Vector3f& posUE,
                               const Eigen::Vector3f& eulerUE)
   {
       Eigen::Matrix4d ue_Twc = Eigen::Matrix4d::Identity();
       const Eigen::Matrix3d R =
           unrealEulerToRotationMatrix(eulerUE.x(), eulerUE.y(), eulerUE.z());
       ue_Twc.block<3, 3>(0, 0) = R;
       ue_Twc.block<3, 1>(0, 3) = posUE.cast<double>();

       const Eigen::Matrix4d Twc_d =
           getT_wue_w().inverse() * ue_Twc * getT_c_cue().inverse();
       return Twc_d.cast<float>();
   }


   void saveTrajectoryAsUE_Format(const std::vector<PoseSE3>& trajectory, const std::string& filename)
   {
       std::ofstream ueFile(filename);
       if (!ueFile.is_open()) {
           std::cerr << "Cannot open file: " << filename << std::endl;
           return;
       }

       int index = 0;
       for (const PoseSE3& pose : trajectory)
       {
           Eigen::Matrix4f Twc;
           Twc.setIdentity();
           Twc.block<3,3>(0,0) = pose.get_rotation();
           Twc.block<3,1>(0,3) = pose.get_position();

           Eigen::Vector3f posUE, eulerUE;
           TwcToUEPose(Twc, &posUE, &eulerUE);

           ueFile << index << " "  
               << std::fixed << std::setprecision(8)
               << posUE.x() << " "
               << posUE.y() << " "
               << posUE.z() << " "
               << eulerUE.x() << " "
               << eulerUE.y() << " "
               << eulerUE.z() << std::endl;
           ++index;
       }


       ueFile.close();
       std::cout << "Saved Unreal trajectory to " << filename << " (" << index << " poses)" << std::endl;
   }

   void saveTrajectoryAsTwcFormat(const std::vector<PoseSE3>& trajectory,
                                  const std::string& filename)
   {
       std::ofstream twcFile(filename);
       if (!twcFile.is_open()) {
           std::cerr << "Cannot open file: " << filename << std::endl;
           return;
       }

       twcFile << std::defaultfloat << std::setprecision(17);
       int index = 0;
       for (const PoseSE3& pose : trajectory)
       {
           Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
           Twc.block<3,3>(0,0) = pose.get_rotation().cast<double>();
           Twc.block<3,1>(0,3) = pose.get_position().cast<double>();

           const double timestamp =
               (index < static_cast<int>(imported_times_.size()))
                   ? imported_times_[static_cast<size_t>(index)]
                   : static_cast<double>(index);
           twcFile << timestamp;
           for (int i = 0; i < 4; ++i) {
               for (int j = 0; j < 4; ++j) {
                   twcFile << " " << Twc(i,j)
				   ;
               }
           }
           twcFile << std::endl;
           ++index;
       }


       twcFile.close();
       std::cout << "Saved Twc trajectory to " << filename << " (" << index << " poses)" << std::endl;
   }


   std::vector<PoseSE3> import_trajectory(std::string data_filename, std::string delimiter, bool use_input_yaw=true){
       std::cout << "Import trajectory file: " << data_filename << std::endl;
       imported_times_.clear();
       std::vector<PoseSE3> trajectory;
       std::ifstream data_file(data_filename);


       if (!data_file.is_open()) {
           std::cerr << "Failed to open the file for reading." << std::endl;
           return trajectory;
       }

       bool format_decided = false;
       bool matrix_mode = false;  // true if file is timestamp + 4x4 Twc matrix

       std::string line;
       int count = 0;
      
       while (std::getline(data_file, line)) {
           if (line.empty() || line[0] == '#') { continue; }

           std::vector<std::string> splitted_line = this->loader.split_string(line, delimiter);
           if (!format_decided) {
               // Twc matrix format: timestamp + 16 numbers -> 17 entries minimum
               matrix_mode = splitted_line.size() >= 17;
               format_decided = true;
           }
           try {
               if (matrix_mode) {
                   if (splitted_line.size() < 17) {
                       std::cerr << "Line " << count << " has insufficient entries for Twc matrix: "
                                 << splitted_line.size() << std::endl;
                       ++count;
                       continue;
                   }
                   const double timestamp = std::stod(splitted_line[0]);
                   imported_times_.push_back(timestamp);


                   Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
                   for (int i = 0; i < 4; ++i) {
                       for (int j = 0; j < 4; ++j) {
                           int idx = 1 + i * 4 + j;  // skip timestamp
                           Twc(i, j) = std::stod(splitted_line[idx]);
                       }
                   }
                   Eigen::Vector3f position = Twc.block<3,1>(0,3).cast<float>();
                   Eigen::Matrix3f rotation = Twc.block<3,3>(0,0).cast<float>();
                   trajectory.emplace_back(position, rotation);
               } else {
                   if (splitted_line.size() < 7) {
                       std::cerr << "Line " << count << " has insufficient entries for UE pose: "
                                 << splitted_line.size() << std::endl;
                       ++count;
                       continue;
                   }
                   Eigen::Vector3f ue_position;
                   ue_position << std::stof(splitted_line[1]),
                                  std::stof(splitted_line[2]),
                                  std::stof(splitted_line[3]);
                   Eigen::Vector3f ue_euler;
                   ue_euler << std::stof(splitted_line[4]),
                               std::stof(splitted_line[5]),
                               std::stof(splitted_line[6]);


                   Eigen::Matrix4f Twc = this->UEPoseToTwc(ue_position, ue_euler);
                   Eigen::Vector3f position = Twc.block<3,1>(0,3);
                   Eigen::Matrix3f rotation = Twc.block<3,3>(0,0);
                   trajectory.emplace_back(position, rotation);
               }
           } catch (const std::exception& e) {
               std::cerr << "Error parsing line " << count << ": " << e.what() << std::endl;
               std::cerr << "Line content: " << line << std::endl;
           }

           ++count;
       }


       data_file.close();
       std::cout << "Successfully imported " << trajectory.size() << " poses from " << count << " lines" << std::endl;
       return trajectory;


   }

   // Eigen::Matrix3f Matrix_from_RPY_degree(float x,float y,float z){//XYZ order, //degree
   //     Eigen::Matrix3f R;
   //     R= Eigen::AngleAxisf(x*M_PI/180.0, Eigen::Vector3f::UnitX())
   //      * Eigen::AngleAxisf(y*M_PI/180.0, Eigen::Vector3f::UnitY())
   //      * Eigen::AngleAxisf(z*M_PI/180.0, Eigen::Vector3f::UnitZ());
   //      return R;
   // }

   Eigen::Matrix3f skew(Eigen::Vector3f x){
       Eigen::Matrix3f skew;
       skew << 0, -x[2], x[1],
            x[2], 0, -x[0],
            -x[1], x[0], 0;
       return skew;
   }
   Eigen::Matrix3f exp_map(Eigen::Vector3f phi){
       Eigen::Matrix3f skew_phi_matrix=this->skew(phi);
       Eigen::Matrix3f I=Eigen::Matrix<float, 3, 3>::Identity();
       float phi_norm =phi.norm();
       float sin_phi_norm_div_phi_norm=sin(phi_norm)/phi_norm;
       float one_minus_cos_phi_norm_div_phi_norm_square=(1.0-cos(phi_norm))/(phi_norm*phi_norm);
       Eigen::Matrix3f skew_phi_matrix_square=skew_phi_matrix*skew_phi_matrix;                    
       Eigen::Matrix3f exp_map=I+sin_phi_norm_div_phi_norm*skew_phi_matrix+one_minus_cos_phi_norm_div_phi_norm_square*skew_phi_matrix_square;
       return exp_map;
   }

   // set each cam pose as frame coordinate reference and express pointcould w.r.t that. 
   void populate_local_indexes(Eigen::Vector3f ref_point){
       Eigen::Vector3f pos=ref_point;
       // print_string("ref_point");
       // print_eigen_v(ref_point);
       this->points_list.clear();
       this->points_list_unnormalized.clear();
       std::vector<Eigen::Vector3f> pointcloud_dir;
       std::vector<float> pointcloud_uncertainty;


       if(this->use_direction||this->use_uncertainty){
           this->points_dir_list.clear();
           this->points_uncertainty_list.clear();
           pointcloud_dir=this->loader.get_pointcloud_dir();
           pointcloud_uncertainty=this->loader.get_pointcloud_uncertainty();
       }
       std::vector<Eigen::Vector3f> pointcloud=this->loader.get_pointcloud();
       //for(int i;i<pointcloud.size();i++){


       this->closest=1000000;
       this->furthest=0;
       for (size_t i:this->valid_points){
       //for (Eigen::Vector3f point: this->loader.get_pointcloud()){
           // print_string("point");
           // print_eigen_v(point);
           Eigen::Vector3f point_pos=pointcloud[i]-pos;
           // print_string("point_pos");
           // print_eigen_v(point_pos);
           if (point_pos.norm()>this->furthest)
               this->furthest=point_pos.norm();
           if (point_pos.norm()<this->closest)
               this->closest=point_pos.norm();        
           // print_string("point_pos");
           // print_eigen_v(point_pos);
          
           this->points_list_unnormalized.push_back(point_pos);


           point_pos=point_pos/point_pos.norm();  //candidate to remove
           this->points_list.push_back(point_pos);




           if(this->use_direction||this->use_uncertainty){
               Eigen::Vector3f point_dir=pointcloud_dir[i];
               float point_uncertainty=pointcloud_uncertainty[i];
               if (point_uncertainty>this->max_uncertainty){
                   this->max_uncertainty=point_uncertainty;
               }
               this->points_dir_list.push_back(point_dir);
               this->points_uncertainty_list.push_back(point_uncertainty);
           }


       }
   }

// rotational velocity constraint
   std::vector<Eigen::Vector3f> velocity_finite_differencing_jacobian(std::vector<PoseSE3> starting_trajectory,float step_size){
       std::vector<Eigen::Vector3f> trajecotry_jacobian;
       //suppose rotation velocity difference is   R2R1^T,  R1 is the earlier rotation, R2 is the later rotation
       for (int i =1;i<starting_trajectory.size()-1;i++){ //restric to elements that are not the head or the tail of the vector
           //jacobian as the later rotation
           Eigen::Vector3f K_2=this->v.transpose();
           Eigen::Vector3f C_2=(starting_trajectory[i].get_rotation())*(starting_trajectory[i-1].get_rotation().transpose())*this->v;
           Eigen::Vector3f later_J=get_Jacobian_from_K_and_C(K_2[0],K_2[1],K_2[2],C_2[0],C_2[1],C_2[2]);
           //jacobian as the earlier rotation
           Eigen::Vector3f K_1=(this->v.transpose())*(starting_trajectory[i+1].get_rotation())*(starting_trajectory[i].get_rotation().transpose());
           Eigen::Vector3f C_1=this->v;
           Eigen::Vector3f early_J=get_Jacobian_of_transposed_exp_from_K_and_C(K_1[0],K_1[1],K_1[2],C_1[0],C_1[1],C_1[2]);
           //store into vector

           trajecotry_jacobian.push_back(step_size*(later_J+early_J));
       }
       return trajecotry_jacobian;
   }

   Eigen::Vector3f get_Jacobian_from_K_and_C(float K1,float K2,float K3,float C1,float C2,float C3){
       Eigen::Vector3f J;
       float a=C2*K3-C3*K2;
       float b=C3*K1-C1*K3;
       float c=C1*K2-C2*K1;
       J<<a,b,c;
       return J;      
   }
   Eigen::Vector3f get_Jacobian_of_transposed_exp_from_K_and_C(float K1,float K2,float K3,float C1,float C2,float C3){
       Eigen::Vector3f J;
       float a=C3*K2-C2*K3;
       float b=C1*K3-C3*K1;
       float c=C2*K1-C1*K2;
       J<<a,b,c;
       return J;      
   }  

   void change_valid_points_list(std::vector<size_t> valid_list){
       this->valid_points=valid_list;
   }

   void write_arrow_to_file(Eigen::Vector3f position,Eigen::Vector3f ray_direction){
       this->initial_trajectory_file<<std::to_string(position[0])<<","<<std::to_string(position[1])<<","<<std::to_string(position[2])<<","<<std::to_string(ray_direction[0])<<","<<std::to_string(ray_direction[1])<<","<<std::to_string(ray_direction[2])<<std::endl;
    }

   void write_arrow_to_output_file(Eigen::Vector3f position,Eigen::Vector3f ray_direction){
       this->output_trajectory_file<<std::to_string(position[0])<<","<<std::to_string(position[1])<<","<<std::to_string(position[2])<<","<<std::to_string(ray_direction[0])<<","<<std::to_string(ray_direction[1])<<","<<std::to_string(ray_direction[2])<<std::endl;
   }

 Eigen::Vector3f calculate_FOV_jacobian_for_pose(PoseSE3 pos,int iteration_count){
   this->populate_local_indexes(pos.get_position());
   Eigen::Matrix3f R=pos.get_rotation();
       Eigen::Vector3f aJ_l;
       aJ_l<<0,0,0;
       float residual=0.0;
       int counter=0;
       for(Eigen::Vector3f K_: this->points_list){


           Eigen::Vector3f K=(K_.transpose())*R; //# this -R
           float C1=this->c[0];  //#this->c
           float C2=this->c[1];
           float C3=this->c[2];
           float K1=K[0];
           float K2=K[1];
           float K3=K[2];

           Eigen::Vector3f F_Jacobian=this->get_Jacobian_from_K_and_C(K1,K2,K3,C1,C2,C3); //jacobian function
           Eigen::Vector3f J=F_Jacobian;
      
           if (this->optimize_visibility_sigmoid==true){   //optimize visibility sigmoid
               float u,KTRC;
               u=(K_.transpose())*(R)*this->c;
               KTRC=u;
               residual=residual+KTRC;

               float visibility_alpha;
               if(iteration_count/this->max_iteration<0.05){
                    visibility_alpha=this->visibility_alpha_180;
               }else if(iteration_count/this->max_iteration>=0.05 && iteration_count/this->max_iteration<0.1){
                    visibility_alpha=this->visibility_alpha_90;
               }else if(iteration_count/this->max_iteration>=0.1 && iteration_count/this->max_iteration<0.15){
                    visibility_alpha=this->visibility_alpha_60;
               }else{
                    visibility_alpha=this->visibility_alpha;
               }

               float w=(-1.0)*this->ks*(u-cos(visibility_alpha)); //visibility_alpha, this->ks
               float v=exp(w);
               float coeff=(-1.0)*(pow((1+v),(-2))*v*(0.0-this->ks));
               F_Jacobian=coeff*J;
           }

               // if self.use_direction==True:
           float alpha=4.0;
           if(this->use_direction){

               //  direction_dot=np.matrix(K)*np.matrix(self.direction_list[kk]).transpose()
               float direction_dot=(K.transpose()*this->points_dir_list[counter]);
               //  print("final coeff",1.0-np.abs(direction_dot[0,0]))
               if(this->DEBUG){
                   // std::cout<<"direction_dot is " <<direction_dot<<std::endl;
               }
               //  alpha=alpha*(1.0-np.abs(direction_dot[0,0]))
               alpha=alpha*(1.0-fabs(direction_dot));
               F_Jacobian=F_Jacobian*alpha;
           }

           float uncertainty_alpha=3;
           if(this->use_uncertainty){
                float point_uncertainty=this->points_uncertainty_list[counter];
                uncertainty_alpha=1.0*point_uncertainty/this->max_uncertainty;
                F_Jacobian=F_Jacobian*uncertainty_alpha;
           }

            float distance=this->points_list_unnormalized[counter].norm();
            float distance_weight=1.0-(distance-this->closest)/(this->furthest-this->closest);
            F_Jacobian=F_Jacobian*distance_weight;

       Eigen::Vector3f aJ=F_Jacobian;
       aJ_l=aJ_l+aJ;
       counter++;
       }
       float step_size=(10/(this->points_list.size()*M_PI*2));

       aJ_l=aJ_l*step_size;
       return aJ_l;
 }

   void optimize(bool write_to_file){
       Eigen::Matrix3f R;
       Eigen::Vector3f v__, v_, v;
       saveTrajectoryAsUE_Format(this->trajectory, this->output_initial_trajectory_filename_ue); //should match stamped_twc_ue that we read as input(checking purpose) 
       saveTrajectoryAsTwcFormat(this->trajectory, this->output_initial_trajectory_filename_twc); //should match stamped_twc that we read as input(checking purpose) 

       for (int i =0;i<this->max_iteration;i++){
           std::vector<Eigen::Vector3f> trajecotry_jacobian=this->velocity_finite_differencing_jacobian(this->trajectory,0.5);
           std::cout<<"trajectory size " <<this->trajectory.size()<<std::endl;
           std::cout<<"trajectory jacobain size " <<trajecotry_jacobian.size()<<std::endl;


           //save quivers[0]
           if (write_to_file){
               this->initial_trajectory_file<<std::endl;
               R=this->trajectory[0].get_rotation();
               v_=this->c;
               v_=R*v_;
               this->write_arrow_to_file(this->trajectory[0].get_position(),v_);
           }

			//optimization
			for (int j =0;j<trajecotry_jacobian.size();j++){
				Eigen::Vector3f FOV_Jacobian,combined_Jacobian;
				FOV_Jacobian=calculate_FOV_jacobian_for_pose(trajectory[j+1],i);
				// combined_Jacobian=FOV_Jacobian+trajecotry_jacobian[j];
				combined_Jacobian=FOV_Jacobian;
				if (log_jacobian){
					std::cout<<"iter "<<i<<" pose "<<(j+1)<<" jacobian "<<combined_Jacobian.transpose()
					         <<" norm "<<combined_Jacobian.norm()<<std::endl;
				}
				R=this->exp_map(combined_Jacobian)*(trajectory[j+1].get_rotation());
				// R=this->exp_map(FOV_Jacobian)*(trajectory[j+1].get_rotation()); //overwrite, FOV only
				this->trajectory[j+1].set_rotation(R); //update rotation


               //save optimization step quivers
               Eigen::Vector3f v; v=this->c; v=R*v;
               if (write_to_file){
                   this->write_arrow_to_file(this->trajectory[j+1].get_position(),v); 
               }  
           }

           //save quivers[last?]
           if (write_to_file){
               R=this->trajectory.back().get_rotation();
               v__=this->c;
               v__=R*v__;
               this->write_arrow_to_file(this->trajectory.back().get_position(),v__);
           }
       }


       if (write_to_file){
           for (PoseSE3 pose:this->trajectory){
               Eigen::Matrix3f R=pose.get_rotation();
               v__=this->c;
               v__=R*v__;


               // std::cout<<"v__"<<v__ <<std::endl;
               this->write_arrow_to_output_file(pose.get_position(),v__);

           }
           //NOTE: added
           saveTrajectoryAsUE_Format(this->trajectory, this->output_trajectory_filename_ue);
           saveTrajectoryAsTwcFormat(this->trajectory, this->output_trajectory_filename_twc);
       }
   }

private:
   Eigen::Vector3f v;
   std::vector<PoseSE3> trajectory;
   std::vector<double> imported_times_;
   std::string output_initial_trajectory_filename;
   std::ofstream initial_trajectory_file;
   std::ofstream output_trajectory_file;
   std::string output_initial_trajectory_filename_ue;
   std::string output_initial_trajectory_filename_twc;
   std::string output_trajectory_filename_ue;
   std::string output_trajectory_filename_twc;
   int max_iteration=30;
   CloudLoader loader;
   std::vector<Eigen::Vector3f> points_list;
   std::vector<Eigen::Vector3f> points_list_unnormalized;
   std::vector<Eigen::Vector3f> points_dir_list;
   std::vector<float> points_uncertainty_list;
   double ks=15;
   float visibility_angle=15.0;
   double visibility_alpha=visibility_angle*M_PI/180.0;
   double visibility_alpha_180=179.0*M_PI/180.0;
   double visibility_alpha_90=90.0*M_PI/180.0;
   double visibility_alpha_60=60.0*M_PI/180.0;
   bool optimize_visibility_sigmoid=true;
   Eigen::Vector3f c; //camera principle axi s
   std::ofstream montecarlopointsfile;
   std::ofstream montecarlopointsdirfile;
   std::ofstream pcdfile;
   std::vector<size_t> valid_points;


	bool use_uncertainty=false;
	bool use_direction=false;
	float max_uncertainty=0;
	bool DEBUG=false;
	bool log_jacobian=true;


   float closest;
   float furthest;
};
#endif
