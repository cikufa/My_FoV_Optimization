#ifndef _TRAJECTORYOPMANIFOLD_


#define _TRAJECTORYOPMANIFOLD_


#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <time.h>
#include <cstdlib>
#include <vector>
#include <typeinfo>
#include <chrono>
#include <fstream>
#include <sstream>
#include <cloud_loader.h>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <limits>
#include <voxblox/core/esdf_map.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/core/common.h>


template <typename T> void print(T x)
{
   std::cout<<x<<std::endl;
}


class myTrajectoryOptimizerOnManifold{
public:
   enum StepNormMode {
       kStepNormPoints = 0,
       kStepNormJacobian = 1,
   };
   enum FovNormMode {
       kFovNormNone = 0,
       kFovNormPoints = 1,
       kFovNormVisible = 2,
   };
   // The FOV Jacobian here is formed with K_cam and c_cam (body/camera frame).
   // To update R correctly you must either:
   //  - convert the body-frame delta to world-frame and left-multiply, or
   //  - right-multiply with the body-frame delta.
   // The legacy behavior (left-multiply with body-frame delta) is frame-mismatched
   // and can move the camera away from visibility maxima. We keep it only for
   // regression comparisons.
   enum UpdateFrameMode {
       kUpdateLegacy = 0,  // body-frame Jacobian with left-multiply (legacy/mismatched).
       kUpdateWorld = 1,   // world-frame Jacobian with left-multiply (fixed, default).
       kUpdateBody = 2,    // body-frame Jacobian with right-multiply (fixed).
   };

   myTrajectoryOptimizerOnManifold(std::string quivers_filename,
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
      
       this->myinitialization(quivers_filename, output_initial_trajectory_filename_ue, output_initial_trajectory_filename_twc,
           input_pointcloud_filename,output_pointcloud_filename,use_direction,use_uncertainty,input_direction_and_uncertainty_filename,
           output_pointcloud_dir_filename,input_trajectory_file,output_trajectory_file, output_trajectory_filename_ue, output_trajectory_filename_twc);
   };

   void set_max_iteration(int value) {
       if (value > 0) {
           this->max_iteration = value;
       }
   }

   void set_ks(double value) {
       if (value > 0.0) {
           this->ks = value;
       }
   }

   void set_ks_from_visibility(bool enabled) {
       this->ks_from_visibility = enabled;
   }

   void set_ks_transition_deg(float value) {
       if (value > 0.0f) {
           this->ks_transition_deg = value;
       }
   }

   void set_visibility_angle_deg(float value) {
       if (value > 0.0f) {
           this->visibility_angle = value;
           this->visibility_alpha = value * M_PI / 180.0;
       }
   }

   void set_base_step_scale(float value) {
       if (value > 0.0f) {
           this->base_step_scale = value;
       }
   }

   void set_step_limits_deg(float min_deg, float max_deg) {
       if (min_deg > 0.0f) {
           this->min_step_rad = min_deg * M_PI / 180.0f;
       }
       if (max_deg > 0.0f) {
           this->max_step_rad = max_deg * M_PI / 180.0f;
       }
   }

   void set_fov_schedule_deg(const std::vector<float>& schedule_deg) {
       this->fov_schedule_rad.clear();
       for (float value : schedule_deg) {
           if (value > 0.0f) {
               this->fov_schedule_rad.push_back(value * M_PI / 180.0f);
           }
       }
   }

   void set_trajectory_jacobian_step(float value) {
       if (value >= 0.0f) {
           this->trajectory_jacobian_step = value;
       }
   }

   void set_log_jacobian(bool enabled) {
       this->log_jacobian = enabled;
   }
   void set_debug_log_enabled(bool enabled) {
       this->debug_log_enabled = enabled;
   }
   void set_debug_log_path(const std::string& path) {
       this->debug_log_path = path;
       this->debug_log_enabled = true;
   }
   void set_visible_feature_dump_path(const std::string& path) {
       this->visible_feature_dump_path_ = path;
       this->visible_feature_dump_enabled_ = !path.empty();
       this->maybe_open_visible_feature_dump_file();
   }

   void set_step_norm_mode(const std::string& mode) {
       std::string lower = mode;
       std::transform(lower.begin(), lower.end(), lower.begin(),
                      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
       if (lower == "jacobian" || lower == "jac") {
           this->step_norm_mode = kStepNormJacobian;
       } else if (lower == "points" || lower == "point") {
           this->step_norm_mode = kStepNormPoints;
       }
   }

   void set_fov_norm_mode(const std::string& mode) {
       std::string lower = mode;
       std::transform(lower.begin(), lower.end(), lower.begin(),
                      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
       if (lower == "points" || lower == "point") {
           this->fov_norm_mode = kFovNormPoints;
       } else if (lower == "visible" || lower == "vis") {
           this->fov_norm_mode = kFovNormVisible;
       } else if (lower == "none" || lower == "off") {
           this->fov_norm_mode = kFovNormNone;
       }
   }

   void set_adaptive_step_enabled(bool enabled) {
       this->adaptive_step_enabled = enabled;
   }

   void set_update_frame_mode(const std::string& mode) {
       std::string lower = mode;
       std::transform(lower.begin(), lower.end(), lower.begin(),
                      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
       if (lower == "legacy" || lower == "old") {
           this->update_frame_mode = kUpdateLegacy;
       } else if (lower == "body" || lower == "right") {
           this->update_frame_mode = kUpdateBody;
       } else if (lower == "world" || lower == "left") {
           this->update_frame_mode = kUpdateWorld;
       }
   }

   void set_min_step_decay(float value) {
       if (value >= 0.0f) {
           this->min_step_decay_strength = value;
       }
   }

   void set_max_step_decay(float value) {
       if (value >= 0.0f) {
           this->max_step_decay_strength = value;
       }
   }

   void set_adapt_max_clip_thresh(float value) {
       if (value >= 0.0f) {
           this->adapt_max_clip_thresh = value;
       }
   }

   void set_adapt_min_clip_thresh(float value) {
       if (value >= 0.0f) {
           this->adapt_min_clip_thresh = value;
       }
   }

   void set_adapt_low_max_clip_thresh(float value) {
       if (value >= 0.0f) {
           this->adapt_low_max_clip_thresh = value;
       }
   }

   void set_adapt_low_min_clip_thresh(float value) {
       if (value >= 0.0f) {
           this->adapt_low_min_clip_thresh = value;
       }
   }

   void set_adapt_shrink_factor(float value) {
       if (value > 0.0f) {
           this->adapt_shrink_factor = value;
       }
   }

   void set_adapt_grow_high_min_factor(float value) {
       if (value > 0.0f) {
           this->adapt_grow_high_min_factor = value;
       }
   }

   void set_adapt_grow_low_clip_factor(float value) {
       if (value > 0.0f) {
           this->adapt_grow_low_clip_factor = value;
       }
   }

   void set_adapt_scale_min(float value) {
       if (value > 0.0f) {
           this->adapt_scale_min = value;
           if (this->adapt_scale_min > this->adapt_scale_max) {
               this->adapt_scale_max = this->adapt_scale_min;
           }
       }
   }

   void set_adapt_scale_max(float value) {
       if (value > 0.0f) {
           this->adapt_scale_max = value;
           if (this->adapt_scale_min > this->adapt_scale_max) {
               this->adapt_scale_min = this->adapt_scale_max;
           }
       }
   }

   bool loadEsdfMap(const std::string& esdf_file_path) {
       try {
           voxblox::Layer<voxblox::EsdfVoxel>::Ptr layer_ptr;
           constexpr bool kMultipleLayerSupport = true;
           if (!voxblox::io::LoadLayer<voxblox::EsdfVoxel>(
                   esdf_file_path, kMultipleLayerSupport, &layer_ptr)) {
               std::cerr << "Failed to load ESDF layer from: " << esdf_file_path << std::endl;
               return false;
           }
           this->esdf_map_.reset(new voxblox::EsdfMap(layer_ptr));
           this->esdf_loaded_ = true;
           this->esdf_map_path_ = esdf_file_path;
           this->initializeRaycastDefaultsFromMap();
           this->invalidateVisibilityCache();
           std::cout << "Successfully loaded ESDF map from: " << esdf_file_path << std::endl;
           return true;
       } catch (const std::exception& e) {
           std::cerr << "Exception loading ESDF map: " << e.what() << std::endl;
           this->esdf_loaded_ = false;
           return false;
       }
   }

   size_t prefilterVisiblePoints() {
       if (!this->esdf_loaded_) {
           std::cerr << "Warning: ESDF map not loaded. Skipping occlusion filtering." << std::endl;
           this->invalidateVisibilityCache();
           return this->valid_points.size();
       }

       if (this->trajectory.empty()) {
           std::cerr << "Warning: Trajectory is empty. Cannot pre-filter visibility." << std::endl;
           this->invalidateVisibilityCache();
           return this->valid_points.size();
       }

       if (this->pointcloud_.empty()) {
           std::cerr << "Warning: Point cloud is empty. Cannot pre-filter visibility." << std::endl;
           this->invalidateVisibilityCache();
           return 0;
       }

       this->visible_points_per_pose_.clear();
       this->visible_points_per_pose_.resize(this->trajectory.size());

       std::vector<char> seen(this->pointcloud_.size(), 0);
       size_t unique_visible_count = 0;
       size_t min_visible = this->valid_points.size();
       size_t max_visible = 0;
       size_t total_visible = 0;

       for (size_t pose_idx = 0; pose_idx < this->trajectory.size(); ++pose_idx) {
           const Eigen::Vector3f camera_pos = this->trajectory[pose_idx].get_position();
           std::vector<size_t>& pose_visible = this->visible_points_per_pose_[pose_idx];
           pose_visible.reserve(this->valid_points.size());

           for (size_t idx : this->valid_points) {
               if (idx >= this->pointcloud_.size()) {
                   continue;
               }
               if (!this->isPointVisible(camera_pos, this->pointcloud_[idx])) {
                   continue;
               }
               pose_visible.push_back(idx);
               if (!seen[idx]) {
                   seen[idx] = 1;
                   unique_visible_count++;
               }
           }

           min_visible = std::min(min_visible, pose_visible.size());
           max_visible = std::max(max_visible, pose_visible.size());
           total_visible += pose_visible.size();
       }

       this->visibility_cache_ready_ = true;

       const double avg_visible =
           this->trajectory.empty()
               ? 0.0
               : static_cast<double>(total_visible) / static_cast<double>(this->trajectory.size());
       std::cout << "Per-pose visibility cache built from " << this->valid_points.size()
                 << " landmarks: unique visible=" << unique_visible_count
                 << ", avg/pose=" << avg_visible
                 << ", min/pose=" << min_visible
                 << ", max/pose=" << max_visible << std::endl;

       return unique_visible_count;
   }

   void setEsdfConfig(float distance_threshold_m, bool use_interpolation = true) {
       if (distance_threshold_m > 0.0f) {
           this->esdf_config_.distance_threshold_m = distance_threshold_m;
       }
       this->esdf_config_.use_interpolation = use_interpolation;
       this->initializeRaycastDefaultsFromMap();
       this->invalidateVisibilityCache();
   }

   void setEsdfRaycastConfig(float ray_step_scale, float ray_min_step_m,
                             float ray_max_step_m, float endpoint_margin_m,
                             bool unknown_is_occluded) {
       if (ray_step_scale > 0.0f) {
           this->esdf_config_.ray_step_scale = ray_step_scale;
       }
       this->esdf_config_.ray_min_step_m = ray_min_step_m;
       this->esdf_config_.ray_max_step_m = ray_max_step_m;
       this->esdf_config_.endpoint_margin_m = endpoint_margin_m;
       this->esdf_config_.unknown_is_occluded = unknown_is_occluded;
       this->initializeRaycastDefaultsFromMap();
       this->invalidateVisibilityCache();
   }

   bool isEsdfLoaded() const {
       return this->esdf_loaded_;
   }

   bool isPointVisible(const Eigen::Vector3f& camera_pos,
                       const Eigen::Vector3f& point_pos) {
       if (!this->esdf_loaded_ || !this->esdf_map_) {
           return true;
       }
       return this->checkVisibilityEsdf(camera_pos, point_pos);
   }

   void myinitialization(
   const std::string& quivers_filename,
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

       this->quivers_filename=quivers_filename;
	   this->output_initial_trajectory_filename_ue = output_initial_trajectory_filename_ue;
       this->output_initial_trajectory_filename_twc = output_initial_trajectory_filename_twc;
       if (!this->quivers_filename.empty()) {
           this->initial_trajectory_file.open(this->quivers_filename);
           if (this->initial_trajectory_file.is_open()) {
               this->initial_trajectory_file
                   << "# columns: x,y,z,dx,dy,dz,visible_count,visibility_score\n"
                   << "# blocks: blank-line separated iterations\n";
           }
       }
       this->maybe_open_visible_feature_dump_file();

       if (!output_trajectory_filename.empty()) {
	       this->output_trajectory_file.open(output_trajectory_filename);
       }
       this->output_trajectory_filename_ue= output_trajectory_filename_ue;
       this->output_trajectory_filename_twc= output_trajectory_filename_twc;


       this->v<<1,1,1;
       this->v=this->v/this->v.norm();
       // Optical axis convention: for Twc in this dataset the camera forward
       // axis aligns with +Z. Using +X here rotates visibility about the wrong
       // axis and degrades optimization.
       this->c<<0,0,1;
       this->c=this->c/this->c.norm();
       this->pcdfile.open(output_pointcloud_filename);
       this->trajectory= this->import_trajectory(input_trajectory_file_name," ", true);
    //    std::cout<<this->trajectory[1].get_position()<<std::endl;
    //    std::cout<<this->trajectory[1].get_rotation()<<std::endl;
       Eigen::Vector3f starting_position;
       // starting_position<<this->trajectory->position;
       // starting_position = this->trajectory[0].get_position();
      
       //NOTE: starting_rotation<<this->trajectory->rotation; //change to robot orientation along path yaw = arctan2(deltay/deltax)
       // Eigen::Matrix3f starting_rotation;
       // starting_rotation<<1,0,0,0,1,0,0,0,1;
       // starting_rotation = this->trajectory[0].get_rotation();

       this->loader.ImportFromXyzFile(input_pointcloud_filename,1,true,false," ");
       this->pointcloud_ = this->loader.get_pointcloud();
       // if(this->use_direction||this->use_uncertainty){
       //      this->loader.ImportFromDirUncertaintyFile(input_direction_and_uncertainty_filename,1," ");
       // }
       //save pointcloud
       for (const Eigen::Vector3f& point : this->pointcloud_){
           this->pcdfile<<point[0]<<","<<point[1]<<","<<point[2]<<std::endl;
       }
       this->pcdfile.close();
    //    std::cout<<"pcd entries written: "<<this->loader.get_pointcloud().size()<<std::endl;


       //save pointcloud_dir and uncertainty
       // std::vector<Eigen::Vector3f> pointcloud_dir_vector=this->loader.get_pointcloud_dir();
       // std::vector<float> points_uncertainty_vector=this->loader.get_pointcloud_uncertainty();
       // for(int i=0;i<pointcloud_dir_vector.size();i++){
       //   this->montecarlopointsdirfile<<pointcloud_dir_vector[i][0]<<","<<pointcloud_dir_vector[i][1]<<","<<pointcloud_dir_vector[i][2]<<","<<points_uncertainty_vector[i]<<std::endl;
       // }
       // this->montecarlopointsdirfile.close();


       this->valid_points.clear();
       this->invalidateVisibilityCache();

       for (size_t i=0;i<this->pointcloud_.size();i++){
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
       if (filename.empty()) {
           return;
       }
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
    //    std::cout << "Saved Unreal trajectory to " << filename << " (" << index << " poses)" << std::endl;
   }

   void saveTrajectoryAsTwcFormat(const std::vector<PoseSE3>& trajectory,
                                  const std::string& filename)
   {
       if (filename.empty()) {
           return;
       }
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
    //    std::cout << "Saved Twc trajectory to " << filename << " (" << index << " poses)" << std::endl;
   }

   std::vector<PoseSE3> import_trajectory(std::string data_filename, std::string delimiter, bool use_input_yaw=true){
    //    std::cout << "Import trajectory file: " << data_filename << std::endl;
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
    //    std::cout << "Successfully imported " << trajectory.size() << " poses from " << count << " lines" << std::endl;
       return trajectory;


   }
   
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
   void populate_local_indexes_for_pose(size_t pose_index){
       const Eigen::Vector3f pos = this->trajectory[pose_index].get_position();
       this->points_list.clear();
       this->points_list_unnormalized.clear();
       std::vector<Eigen::Vector3f> pointcloud_dir;
       std::vector<float> pointcloud_uncertainty;


       if(this->use_direction||this->use_uncertainty){
           this->points_dir_list.clear();
           this->points_uncertainty_list.clear();
           this->max_uncertainty = 0.0f;
           pointcloud_dir=this->loader.get_pointcloud_dir();
           pointcloud_uncertainty=this->loader.get_pointcloud_uncertainty();
       }
       this->points_index_list.clear();
       const std::vector<size_t>& pose_valid_points = this->getValidPointsForPose(pose_index);


       this->closest=std::numeric_limits<float>::max();
       this->furthest=0.0f;
       for (size_t i:pose_valid_points){
       //for (Eigen::Vector3f point: this->loader.get_pointcloud()){
           // print_string("point");
           // print_eigen_v(point);
           if (i >= this->pointcloud_.size()) {
               continue;
           }
           Eigen::Vector3f point_pos=this->pointcloud_[i]-pos;
           // print_string("point_pos");
           // print_eigen_v(point_pos);
           const float point_distance = point_pos.norm();
           if (point_distance <= 1e-6f) {
               continue;
           }
           if (point_distance>this->furthest)
               this->furthest=point_distance;
           if (point_distance<this->closest)
               this->closest=point_distance;        
           // print_string("point_pos");
           // print_eigen_v(point_pos);
          
           this->points_list_unnormalized.push_back(point_pos);
           point_pos=point_pos/point_distance; 
           this->points_list.push_back(point_pos);
           this->points_index_list.push_back(i);

           if(this->use_direction||this->use_uncertainty){
               Eigen::Vector3f point_dir = Eigen::Vector3f::Zero();
               float point_uncertainty = 0.0f;
               if (i < pointcloud_dir.size()) {
                   point_dir = pointcloud_dir[i];
               }
               if (i < pointcloud_uncertainty.size()) {
                   point_uncertainty = pointcloud_uncertainty[i];
               }
               if (point_uncertainty>this->max_uncertainty){
                   this->max_uncertainty=point_uncertainty;
               }
               this->points_dir_list.push_back(point_dir);
               this->points_uncertainty_list.push_back(point_uncertainty);
           }
       }
       if (this->closest == std::numeric_limits<float>::max()) {
           this->closest = 0.0f;
       }
   }

// rotational velocity constraint
   std::vector<Eigen::Vector3f> velocity_finite_differencing_jacobian(std::vector<PoseSE3> starting_trajectory){
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

           trajecotry_jacobian.push_back(later_J+early_J);
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
       this->invalidateVisibilityCache();
   }

   std::vector<size_t> get_visible_feature_indices_for_pose(size_t pose_index,
                                                            float visibility_alpha_rad){
       std::vector<size_t> visible_indices;
       if (pose_index >= this->trajectory.size()) {
           return visible_indices;
       }
       this->populate_local_indexes_for_pose(pose_index);
       if (this->points_list.empty()) {
           return visible_indices;
       }
       visible_indices.reserve(this->points_list.size());
       const Eigen::Matrix3f R = this->trajectory[pose_index].get_rotation();
       const float cos_alpha = std::cos(visibility_alpha_rad);
       const size_t count =
           std::min(this->points_list.size(), this->points_index_list.size());
       for (size_t idx = 0; idx < count; ++idx) {
           const Eigen::Vector3f& K_ = this->points_list[idx];
           const float u = (K_.transpose()) * R * this->c;
           if (u >= cos_alpha) {
               visible_indices.push_back(this->points_index_list[idx]);
           }
       }
       return visible_indices;
   }

   void write_arrow_to_file(Eigen::Vector3f position,Eigen::Vector3f ray_direction){
       if (!this->initial_trajectory_file.is_open()) {
           return;
       }
       this->initial_trajectory_file<<std::to_string(position[0])<<","<<std::to_string(position[1])<<","<<std::to_string(position[2])<<","<<std::to_string(ray_direction[0])<<","<<std::to_string(ray_direction[1])<<","<<std::to_string(ray_direction[2])<<std::endl;
    }

   void write_arrow_to_output_file(Eigen::Vector3f position,Eigen::Vector3f ray_direction){
       if (!this->output_trajectory_file.is_open()) {
           return;
       }
       this->output_trajectory_file<<std::to_string(position[0])<<","<<std::to_string(position[1])<<","<<std::to_string(position[2])<<","<<std::to_string(ray_direction[0])<<","<<std::to_string(ray_direction[1])<<","<<std::to_string(ray_direction[2])<<std::endl;
   }

   void write_arrow_with_metrics(std::ofstream& file, Eigen::Vector3f position,
                                 Eigen::Vector3f ray_direction,
                                 float visible_count, float visibility_score){
       if (!file.is_open()) {
           return;
       }
       file<<std::to_string(position[0])<<","<<std::to_string(position[1])<<","<<std::to_string(position[2])<<","
           <<std::to_string(ray_direction[0])<<","<<std::to_string(ray_direction[1])<<","<<std::to_string(ray_direction[2])<<","
           <<std::to_string(visible_count)<<","<<std::to_string(visibility_score)
           <<std::endl;
   }

   Eigen::Vector3f calculate_FOV_jacobian_for_pose(size_t pose_index,int iteration_count,
                                                   float* visible_count = nullptr,
                                                   float* visibility_score = nullptr){
    this->populate_local_indexes_for_pose(pose_index);
    Eigen::Matrix3f R=this->trajectory[pose_index].get_rotation();
        Eigen::Vector3f aJ_l;
        aJ_l<<0,0,0;
        float residual=0.0;
        int counter=0;
        const double ks_iter = GetKsForIteration(iteration_count);
        const float visibility_alpha = GetVisibilityAlpha(iteration_count);
        const float cos_alpha = std::cos(visibility_alpha);
        float vis_count_local = 0.0f;
        float vis_score_local = 0.0f;
        for(Eigen::Vector3f K_: this->points_list){
            Eigen::Vector3f K=(K_.transpose())*R; //# this -R
            float C1=this->c[0];  
            float C2=this->c[1];
            float C3=this->c[2];
            float K1=K[0];
            float K2=K[1];
            float K3=K[2];  //NOTE: c nabayad R* this->c bashe inja ke mese paper beshe ? Na doroste

            Eigen::Vector3f F_Jacobian=this->get_Jacobian_from_K_and_C(K1,K2,K3,C1,C2,C3); //jacobian function return c x k
            Eigen::Vector3f J=F_Jacobian;

            const float u = (K_.transpose())*(R)*this->c;
            if (visible_count) {
                if (u >= cos_alpha) {
                    vis_count_local += 1.0f;
                }
            }
            if (visibility_score) {
                float score = 0.0f;
                if (this->optimize_visibility_sigmoid) {
                    const float w = (-1.0f) * static_cast<float>(ks_iter) * (u - cos_alpha);
                    score = 1.0f / (1.0f + std::exp(w));
                } else {
                    score = (u >= cos_alpha) ? 1.0f : 0.0f;
                }
                vis_score_local += score;
            }
        
            if (this->optimize_visibility_sigmoid==true){   //optimize visibility sigmoid
                float KTRC;
                KTRC=u;
                residual=residual+KTRC;

                float w=(-1.0f)*static_cast<float>(ks_iter)*(u-cos_alpha);
                float v=exp(w);
                float coeff=(-1.0f)*(pow((1+v),(-2))*v*(0.0f-static_cast<float>(ks_iter)));
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
                    const float max_uncertainty =
                        std::max(1e-6f, this->max_uncertainty);
                    uncertainty_alpha=1.0f*point_uncertainty/max_uncertainty;
                    F_Jacobian=F_Jacobian*uncertainty_alpha;
            }

                const float distance=this->points_list_unnormalized[counter].norm();
                const float distance_span = std::max(1e-6f, this->furthest-this->closest);
                const float distance_weight=1.0f-(distance-this->closest)/distance_span;
                F_Jacobian=F_Jacobian*distance_weight;

        Eigen::Vector3f aJ=F_Jacobian;
        aJ_l=aJ_l+aJ;
        counter++;
        }
        if (visible_count) {
            *visible_count = vis_count_local;
        }
        if (visibility_score) {
            *visibility_score = vis_score_local;
        }
        if (this->fov_norm_mode == kFovNormPoints) {
            const float denom = std::max(1.0f, static_cast<float>(this->points_list.size()));
            aJ_l = 5* aJ_l / (denom *M_PI);  //changed 
        } else if (this->fov_norm_mode == kFovNormVisible) {
            const float denom = std::max(1.0f, vis_count_local);
            aJ_l = aJ_l / denom;
        }
        return aJ_l;
    }

   void calculate_visibility_metrics_for_pose(size_t pose_index, int iteration_count,
                                              float visibility_alpha_rad,
                                              float* visible_count,
                                              float* visibility_score,
                                              double visibility_ks = -1.0) {
       if (!visible_count && !visibility_score) {
           return;
       }
       this->populate_local_indexes_for_pose(pose_index);
       Eigen::Matrix3f R = this->trajectory[pose_index].get_rotation();
       const double ks_iter =
           (visibility_ks > 0.0) ? visibility_ks : GetKsForIteration(iteration_count);
       const float cos_alpha = std::cos(visibility_alpha_rad);
       float vis_count_local = 0.0f;
       float vis_score_local = 0.0f;
       for (Eigen::Vector3f K_ : this->points_list) {
           const float u = (K_.transpose()) * R * this->c;
           if (visible_count && u >= cos_alpha) {
               vis_count_local += 1.0f;
           }
           if (visibility_score) {
               float score = 0.0f;
               if (this->optimize_visibility_sigmoid) {
                   const float w =
                       (-1.0f) * static_cast<float>(ks_iter) * (u - cos_alpha);
                   score = 1.0f / (1.0f + std::exp(w));
               } else {
                   score = (u >= cos_alpha) ? 1.0f : 0.0f;
               }
               vis_score_local += score;
           }
       }
       if (visible_count) {
           *visible_count = vis_count_local;
       }
       if (visibility_score) {
           *visibility_score = vis_score_local;
       }
   }

   void optimize(bool write_to_file){
       Eigen::Matrix3f R;
       Eigen::Vector3f v__, v_, v;
       this->maybe_open_visible_feature_dump_file();
       if (this->esdf_loaded_ && !this->visibility_cache_ready_) {
           this->prefilterVisiblePoints();
       }
       saveTrajectoryAsUE_Format(this->trajectory, this->output_initial_trajectory_filename_ue); //should match stamped_twc_ue that we read as input(checking purpose) 
       saveTrajectoryAsTwcFormat(this->trajectory, this->output_initial_trajectory_filename_twc); //should match stamped_twc that we read as input(checking purpose) 

       std::ofstream debug_log;
       bool debug_log_active = false;
       if (this->debug_log_enabled) {
           std::string log_path;
           if (!this->debug_log_path.empty()) {
               if (HasPathSeparator(this->debug_log_path)) {
                   log_path = this->debug_log_path;
               } else {
                   const std::string base = DebugBasePath();
                   if (!base.empty()) {
                       log_path = JoinPath(base, this->debug_log_path);
                   } else {
                       log_path = this->debug_log_path;
                   }
               }
           } else {
               log_path = DefaultDebugLogPath();
           }
           if (!log_path.empty()) {
               debug_log.open(log_path);
               if (debug_log.is_open()) {
                   debug_log_active = true;
                   debug_log << "iter,pose_idx,pos_x,pos_y,pos_z,"
                              << "yaw_deg,pitch_deg,roll_deg,"
                              << "fov_jac_x,fov_jac_y,fov_jac_z,"
                              << "traj_jac_x,traj_jac_y,traj_jac_z,"
                              << "jac_norm,base_step,target_delta_norm,step,"
                              << "clipped_min,clipped_max,adaptive_step_scale,"
                              << "min_step_iter,max_step_iter,norm_scale,alpha_deg,"
                              << "vis_count_sched,vis_score_sched,"
                              << "vis_count_post,vis_score_post,"
                              << "vis_count_fixed,vis_score_fixed"
                              << std::endl;
               }
           }
       }

       const float points_count =
           std::max(1.0f, static_cast<float>(this->valid_points.size()));
       float prev_avg_jac_norm = 1.0f;
       float adaptive_step_scale = 1.0f;
       for (int i =0;i<this->max_iteration;i++){
           std::vector<Eigen::Vector3f> trajecotry_jacobian=
                this->velocity_finite_differencing_jacobian(this->trajectory); //this->trajectory_jacobian_step: scale factor for the smoothness (velocity) Jacobian
        //    std::cout<<"trajectory size " <<this->trajectory.size()<<std::endl;
        //    std::cout<<"trajectory jacobain size " <<trajecotry_jacobian.size()<<std::endl;

           int clipped_min_count = 0;
           int clipped_max_count = 0;
           int active_update_count = 0;
           float jac_norm_sum = 0.0f;
           int jac_norm_count = 0;
           const float iter_ratio =
               static_cast<float>(i) / std::max(1, this->max_iteration - 1);
           const float min_step_iter =
               std::max(0.0f,
                        this->min_step_rad *
                            (1.0f - this->min_step_decay_strength * iter_ratio));
           const float max_step_iter =
               std::max(min_step_iter,
                        this->max_step_rad *
                            (1.0f - this->max_step_decay_strength * iter_ratio));
           const float norm_scale_iter =
               (this->step_norm_mode == kStepNormJacobian)
                   ? std::max(1e-6f, prev_avg_jac_norm)
                   : 1.0f;


			//optimization
			for (int j =0;j<trajecotry_jacobian.size();j++){
				Eigen::Vector3f FOV_Jacobian,combined_Jacobian;
				float vis_count = 0.0f;
				float vis_score = 0.0f;
				FOV_Jacobian=calculate_FOV_jacobian_for_pose(static_cast<size_t>(j + 1),i,
                                                        &vis_count, &vis_score);
                float norm_scale = norm_scale_iter;
                float smooth_scale = 0.5f;
                const Eigen::Vector3f traj_scaled =
                    this->trajectory_jacobian_step * smooth_scale * trajecotry_jacobian[j];
				combined_Jacobian = FOV_Jacobian + traj_scaled;
				R = trajectory[j+1].get_rotation();

				const float jacobian_norm = combined_Jacobian.norm();
				float base_step = 0.0f;
				float target_delta_norm = 0.0f;
				float step = 0.0f;
				bool clipped_min = false;
				bool clipped_max = false;
				if (jacobian_norm > 1e-9f) {
                    jac_norm_sum += jacobian_norm;
                    jac_norm_count++;
                    active_update_count++;
                    base_step =
                        (adaptive_step_scale * this->base_step_scale) / norm_scale;
					target_delta_norm = base_step * jacobian_norm;
					if (target_delta_norm < min_step_iter) {
						target_delta_norm = min_step_iter;
                        clipped_min_count++;
                        clipped_min = true;
					}
					if (target_delta_norm > max_step_iter) {
						target_delta_norm = max_step_iter;
                        clipped_max_count++;
                        clipped_max = true;
					}
                    step = target_delta_norm / jacobian_norm;
					const Eigen::Vector3f delta_body = step * combined_Jacobian;
					if (this->update_frame_mode == kUpdateLegacy) {
						// Legacy/mismatched: left-multiply with body-frame delta.
						R = this->exp_map(delta_body) * R;
					} else if (this->update_frame_mode == kUpdateBody) {
						// Body-frame update: right-multiply.
						R = R * this->exp_map(delta_body);
					} else {
						// World-frame update: convert body delta to world and left-multiply.
						const Eigen::Vector3f delta_world = R * delta_body;
						R = this->exp_map(delta_world) * R;
					}
                    this->trajectory[j+1].set_rotation(R); //update rotation
				}


				if (debug_log_active) {
                    float vis_count_post = 0.0f;
                    float vis_score_post = 0.0f;
                    float vis_count_fixed = 0.0f;
                    float vis_score_fixed = 0.0f;
                    const float alpha_rad = GetVisibilityAlpha(i);
                    this->calculate_visibility_metrics_for_pose(
                        static_cast<size_t>(j + 1), i, alpha_rad,
                        &vis_count_post, &vis_score_post);
                    constexpr float kFixedAlphaRad = 30.0f * static_cast<float>(M_PI) / 180.0f;
                    this->calculate_visibility_metrics_for_pose(
                        static_cast<size_t>(j + 1), i, kFixedAlphaRad,
                        &vis_count_fixed, &vis_score_fixed);

                   const Eigen::Vector3f pos = this->trajectory[j+1].get_position();
                   double yaw = 0.0, pitch = 0.0, roll = 0.0;
                   const Eigen::Quaterniond q(R.cast<double>());
                   quaternionToEulerUnrealEngine(q, yaw, pitch, roll);
                   const float alpha_deg = alpha_rad * 180.0f / static_cast<float>(M_PI);
                   debug_log << i << "," << (j + 1) << ","
                             << pos.x() << "," << pos.y() << "," << pos.z() << ","
                             << yaw << "," << pitch << "," << roll << ","
                             << FOV_Jacobian.x() << "," << FOV_Jacobian.y() << "," << FOV_Jacobian.z() << ","
                             << traj_scaled.x() << ","
                             << traj_scaled.y() << ","
                             << traj_scaled.z() << ","
                             << jacobian_norm << "," << base_step << "," << target_delta_norm << "," << step << ","
                             << (clipped_min ? 1 : 0) << "," << (clipped_max ? 1 : 0) << "," << adaptive_step_scale << ","
                              << min_step_iter << "," << max_step_iter << "," << norm_scale << ","
                              << alpha_deg << ","
                              << vis_count << "," << vis_score << ","
                              << vis_count_post << "," << vis_score_post << ","
                              << vis_count_fixed << "," << vis_score_fixed
                              << std::endl;
                }
           }

           if (write_to_file){
               if (this->initial_trajectory_file.is_open()) {
                   if (i > 0) {
                       this->initial_trajectory_file << std::endl;
                   }
                   this->initial_trajectory_file << "# iteration " << i << std::endl;
               }
               if (this->visible_feature_dump_file_.is_open()) {
                   if (i > 0) {
                       this->visible_feature_dump_file_ << std::endl;
                   }
                   this->visible_feature_dump_file_ << "# iteration " << i << std::endl;
               }
               const float export_alpha_rad = GetMetricVisibilityAlpha();
               const double export_ks = GetMetricKs();
               for (size_t k = 0; k < this->trajectory.size(); ++k) {
                   R = this->trajectory[k].get_rotation();
                   v_ = this->c;
                   v_ = R * v_;
                   if (this->initial_trajectory_file.is_open()) {
                       float vis_count = 0.0f;
                       float vis_score = 0.0f;
                       this->calculate_visibility_metrics_for_pose(
                           k, i, export_alpha_rad,
                           &vis_count, &vis_score, export_ks);
                       this->write_arrow_with_metrics(this->initial_trajectory_file,
                                                      this->trajectory[k].get_position(), v_,
                                                      vis_count, vis_score);
                   }
                   if (this->visible_feature_dump_file_.is_open()) {
                       const std::vector<size_t> visible_idx =
                           this->get_visible_feature_indices_for_pose(k, export_alpha_rad);
                       this->visible_feature_dump_file_ << k;
                       for (size_t idx : visible_idx) {
                           this->visible_feature_dump_file_ << " " << idx;
                       }
                       this->visible_feature_dump_file_ << std::endl;
                   }
               }
           }

           if (this->step_norm_mode == kStepNormJacobian && jac_norm_count > 0) {
               prev_avg_jac_norm = jac_norm_sum / jac_norm_count;
           }

           if (this->adaptive_step_enabled && active_update_count > 0) {
               const float min_clip_ratio =
                   static_cast<float>(clipped_min_count) / active_update_count;
               const float max_clip_ratio =
                   static_cast<float>(clipped_max_count) / active_update_count;
               if (max_clip_ratio > this->adapt_max_clip_thresh) {
                   adaptive_step_scale = std::max(
                       this->adapt_scale_min,
                       adaptive_step_scale * this->adapt_shrink_factor);
               } else if (min_clip_ratio > this->adapt_min_clip_thresh) {
                   adaptive_step_scale =
                       std::min(this->adapt_scale_max,
                                adaptive_step_scale *
                                    this->adapt_grow_high_min_factor);
               } else if (max_clip_ratio < this->adapt_low_max_clip_thresh &&
                          min_clip_ratio < this->adapt_low_min_clip_thresh) {
                   adaptive_step_scale =
                       std::min(this->adapt_scale_max,
                                adaptive_step_scale *
                                    this->adapt_grow_low_clip_factor);
               }
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
   float GetMetricVisibilityAlpha() const {
       if (!this->fov_schedule_rad.empty()) {
           return this->fov_schedule_rad.back();
       }
       return this->visibility_alpha;
   }

   float GetVisibilityAlpha(int iteration_count) const {
       if (!this->fov_schedule_rad.empty()) {
           const int stages = static_cast<int>(this->fov_schedule_rad.size());
           const int stage_len = std::max(1, this->max_iteration / stages);
           int stage_idx = iteration_count / stage_len;
           if (stage_idx >= stages) {
               stage_idx = stages - 1;
           }
           return this->fov_schedule_rad[stage_idx];
       }
       const float ratio = static_cast<float>(iteration_count) /
                           std::max(1, this->max_iteration);
    //    if (ratio < 0.1f) {
    //        return this->visibility_alpha_180;
    //    }
       if (ratio < 0.1f) {
           return this->visibility_alpha_90;
       }
       if (ratio < 0.15f) {
           return this->visibility_alpha_60;
       }
        if (ratio < 0.2f) {
              return this->visibility_alpha_30;
       }
       return this->visibility_alpha;
   }

   double GetKsForVisibilityAlpha(float alpha) const {
       if (!this->ks_from_visibility || this->ks_transition_deg <= 0.0f) {
           return this->ks;
       }
       const double sin_alpha = std::sin(static_cast<double>(alpha));
       if (std::abs(sin_alpha) < 1e-6) {
           return this->ks;
       }
       const double delta_rad = static_cast<double>(this->ks_transition_deg) * M_PI / 180.0;
       if (delta_rad <= 0.0) {
           return this->ks;
       }
       const double dyn = 2.94 / (delta_rad * sin_alpha);
       if (!std::isfinite(dyn) || dyn <= 0.0) {
           return this->ks;
       }
       return dyn;
   }

   double GetMetricKs() const {
       return GetKsForVisibilityAlpha(GetMetricVisibilityAlpha());
   }

   double GetKsForIteration(int iteration_count) const {
       return GetKsForVisibilityAlpha(GetVisibilityAlpha(iteration_count));
   }

   bool checkVisibilityEsdf(const Eigen::Vector3f& camera_pos,
                            const Eigen::Vector3f& point_pos) const {
       if (!this->esdf_map_) {
           return true;
       }

       double dist_camera = 0.0;
       double dist_point = 0.0;

       const bool camera_query_valid = this->esdf_map_->getDistanceAtPosition(
           camera_pos.cast<double>(),
           this->esdf_config_.use_interpolation,
           &dist_camera);
       const bool point_query_valid = this->esdf_map_->getDistanceAtPosition(
           point_pos.cast<double>(),
           this->esdf_config_.use_interpolation,
           &dist_point);

       if (!camera_query_valid || !point_query_valid) {
           return !this->esdf_config_.unknown_is_occluded;
       }

       if (!std::isfinite(dist_camera) || !std::isfinite(dist_point)) {
           return !this->esdf_config_.unknown_is_occluded;
       }

       const float threshold = this->esdf_config_.distance_threshold_m;
       if (static_cast<float>(dist_camera) <= threshold ||
           static_cast<float>(dist_point) <= threshold) {
           return false;
       }

       const Eigen::Vector3d src = camera_pos.cast<double>();
       const Eigen::Vector3d dst = point_pos.cast<double>();
       const Eigen::Vector3d delta = dst - src;
       const double total_len = delta.norm();
       const float endpoint_margin = std::max(0.0f, this->esdf_config_.endpoint_margin_m);
       if (total_len <= endpoint_margin) {
           return false;
       }

       const double ray_min_step =
           std::max(1e-3f, this->esdf_config_.ray_min_step_m);
       const double ray_max_step =
           std::max(static_cast<double>(ray_min_step),
                    static_cast<double>(this->esdf_config_.ray_max_step_m));
       const Eigen::Vector3d dir = delta / total_len;
       double t = ray_min_step;
       const double t_end = total_len - endpoint_margin;
       while (t < t_end) {
           const Eigen::Vector3d sample = src + dir * t;
           double dist_sample = 0.0;
           const bool observed = this->esdf_map_->getDistanceAtPosition(
               sample, this->esdf_config_.use_interpolation, &dist_sample);
           if (!observed || !std::isfinite(dist_sample)) {
               return !this->esdf_config_.unknown_is_occluded;
           }
           if (dist_sample <= threshold) {
               return false;
           }

           double step = std::max(
               ray_min_step,
               static_cast<double>(dist_sample) * this->esdf_config_.ray_step_scale);
           step = std::min(step, ray_max_step);
           t += step;
       }

       return true;
   }

   void initializeRaycastDefaultsFromMap() {
       if (!this->esdf_map_) {
           return;
       }
       const float voxel_size = static_cast<float>(this->esdf_map_->voxel_size());
       if (voxel_size <= 0.0f) {
           return;
       }
       if (this->esdf_config_.ray_min_step_m <= 0.0f) {
           this->esdf_config_.ray_min_step_m = 0.5f * voxel_size;
       }
       if (this->esdf_config_.ray_max_step_m <= 0.0f) {
           this->esdf_config_.ray_max_step_m = 2.0f * voxel_size;
       }
       if (this->esdf_config_.endpoint_margin_m <= 0.0f) {
           this->esdf_config_.endpoint_margin_m = 1.5f * voxel_size;
       }
   }

   void invalidateVisibilityCache() {
       this->visible_points_per_pose_.clear();
       this->visibility_cache_ready_ = false;
   }

   void maybe_open_visible_feature_dump_file() {
       if (!this->visible_feature_dump_enabled_ ||
           this->visible_feature_dump_path_.empty() ||
           this->visible_feature_dump_file_.is_open()) {
           return;
       }
       this->visible_feature_dump_file_.open(this->visible_feature_dump_path_);
       if (this->visible_feature_dump_file_.is_open()) {
           this->visible_feature_dump_file_
               << "# format: one iteration block per blank-line-separated section\n"
               << "# each non-comment line: pose_index global_point_index ...\n";
           return;
       }
       std::cerr << "Warning: failed to open visible feature dump file: "
                 << this->visible_feature_dump_path_ << std::endl;
       this->visible_feature_dump_enabled_ = false;
   }

   const std::vector<size_t>& getValidPointsForPose(size_t pose_index) const {
       if (this->visibility_cache_ready_ &&
           pose_index < this->visible_points_per_pose_.size()) {
           return this->visible_points_per_pose_[pose_index];
       }
       return this->valid_points;
   }

   Eigen::Vector3f v;
   std::vector<PoseSE3> trajectory;
   std::vector<double> imported_times_;
   std::string quivers_filename ;
   std::ofstream initial_trajectory_file;
   std::ofstream output_trajectory_file;
   std::string output_initial_trajectory_filename_ue;
   std::string output_initial_trajectory_filename_twc;
   std::string output_trajectory_filename_ue;
   std::string output_trajectory_filename_twc;
   int max_iteration=30;
   CloudLoader loader;
   std::vector<Eigen::Vector3f> pointcloud_;
   std::vector<Eigen::Vector3f> points_list;
   std::vector<Eigen::Vector3f> points_list_unnormalized;
   std::vector<size_t> points_index_list;
   std::vector<Eigen::Vector3f> points_dir_list;
   std::vector<float> points_uncertainty_list;
   double ks=15;
   bool ks_from_visibility=false;
   float ks_transition_deg=0.0f;
   float visibility_angle=15.0;
   double visibility_alpha=visibility_angle*M_PI/180.0;
//    double visibility_alpha_180=179.0*M_PI/180.0;
   double visibility_alpha_90=90.0*M_PI/180.0;
   double visibility_alpha_60=60.0*M_PI/180.0;
   double visibility_alpha_30=30.0*M_PI/180.0;
   bool optimize_visibility_sigmoid=true;
   Eigen::Vector3f c; //camera principle axi s
   std::ofstream montecarlopointsfile;
   std::ofstream montecarlopointsdirfile;
   std::ofstream pcdfile;
   std::vector<size_t> valid_points;
   std::vector<std::vector<size_t>> visible_points_per_pose_;
   bool visibility_cache_ready_ = false;

   struct EsdfConfig {
       float distance_threshold_m = 0.1f;
       bool use_interpolation = true;
       float ray_step_scale = 0.8f;
       float ray_min_step_m = -1.0f;
       float ray_max_step_m = -1.0f;
       float endpoint_margin_m = -1.0f;
       bool unknown_is_occluded = true;
   };

   EsdfConfig esdf_config_;
   voxblox::EsdfMap::Ptr esdf_map_;
   bool esdf_loaded_ = false;
   std::string esdf_map_path_;


   bool use_uncertainty=false;
   bool use_direction=false;
   float max_uncertainty=0;
   bool DEBUG=false;
   bool log_jacobian=true;
   bool debug_log_enabled=false;
   std::string debug_log_path;
   bool visible_feature_dump_enabled_ = false;
   std::string visible_feature_dump_path_;
   std::ofstream visible_feature_dump_file_;
   int step_norm_mode=kStepNormPoints;
   int fov_norm_mode=kFovNormPoints;
   int update_frame_mode=kUpdateWorld;
   bool adaptive_step_enabled=true;
   float base_step_scale=0.2;
   float min_step_rad=0.25f * M_PI / 180.0f;
   float max_step_rad=5.0f * M_PI / 180.0f;
   float min_step_decay_strength=1.0f;
   float max_step_decay_strength=0.5f;
   float trajectory_jacobian_step=0.5f;
   float adapt_max_clip_thresh=0.35f;
   float adapt_min_clip_thresh=0.80f;
   float adapt_low_max_clip_thresh=0.05f;
   float adapt_low_min_clip_thresh=0.20f;
   float adapt_shrink_factor=0.70f;
   float adapt_grow_high_min_factor=1.15f;
   float adapt_grow_low_clip_factor=1.05f;
   float adapt_scale_min=0.10f;
   float adapt_scale_max=10.0f;
   std::vector<float> fov_schedule_rad;


   float closest;
   float furthest;

   static std::string DirName(const std::string& path) {
       const std::string::size_type pos = path.find_last_of("/\\");
       if (pos == std::string::npos) {
           return std::string();
       }
       return path.substr(0, pos);
   }

   static bool HasPathSeparator(const std::string& path) {
       return path.find('/') != std::string::npos || path.find('\\') != std::string::npos;
   }

   static std::string JoinPath(const std::string& left, const std::string& right) {
       if (left.empty()) {
           return right;
       }
       if (left.back() == '/' || left.back() == '\\') {
           return left + right;
       }
       return left + "/" + right;
   }

   static std::string AppendSuffixBeforeExtension(const std::string& path,
                                                  const std::string& suffix) {
       const std::string::size_type sep = path.find_last_of("/\\");
       const std::string::size_type dot = path.find_last_of('.');
       if (dot != std::string::npos && (sep == std::string::npos || dot > sep)) {
           return path.substr(0, dot) + suffix + path.substr(dot);
       }
       return path + suffix;
   }

   std::string DebugBasePath() const {
       std::string base = output_trajectory_filename_twc;
       if (base.empty()) {
           base = output_trajectory_filename_ue;
       }
       if (base.empty()) {
           return std::string();
       }
       return DirName(base);
   }

   std::string DefaultDebugLogPath() const {
       const std::string dir = DebugBasePath();
       if (dir.empty()) {
           return std::string("optimization_debug.csv");
       }
       return JoinPath(dir, "optimization_debug.csv");
   }
};
#endif
