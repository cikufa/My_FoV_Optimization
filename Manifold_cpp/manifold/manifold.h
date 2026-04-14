// Checks if _ANIMALS IF DECLARED
#ifndef _MANIFOLD_
  
// Defines _ANIMALS_ if above
// conditions fails
#define _MANIFOLD_

#ifdef USE_MANIFOLD_BACKUP
#include "manifold_backup.h"
#else

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <time.h> 
#include <vector>
#include <typeinfo>
#include <chrono>
#include <cmath>
// #include <Eigen/Dense>
#include <fstream> 
#include <sstream>
#include <limits>
#include <cstdlib>
#include <cloud_loader.h>

/*ADDED:*/
#include <eigen3/Eigen/Geometry>
Eigen::Quaternionf matrixToQuaternion(const Eigen::Matrix3f& R) {
    return Eigen::Quaternionf(R).normalized();
}

int print_eigen(Eigen::Matrix3f m)
{
    // Eigen Matrices do have rule to print them with std::cout
    std::cout << m << std::endl;
    return 0;
}

int print_eigen_v(Eigen::Vector3f m)
{
    // Eigen Matrices do have rule to print them with std::cout
    std::cout << m << std::endl;
    return 0;
}

int print_string (std::string s)
{
    std::cout << s << std::endl;
    return 0;	
}

int print_float (float s){
    std::cout << s << std::endl;
    return 0;		
}

inline bool file_is_empty(const std::string& path) {
    std::ifstream in(path, std::ios::ate | std::ios::binary);
    if (!in.good()) {
        return true;
    }
    return in.tellg() == 0;
}

inline void write_header_if_empty(const std::string& path, std::ofstream& stream, const std::string& header) {
    if (!stream.is_open()) {
        return;
    }
    if (file_is_empty(path)) {
        stream << header << std::endl;
    }
}
// class Manifold_minimize_angle_among_all_target_points(object):

class FovOptimizerOnManifold{
public:
		enum UpdateFrameMode {
			kUpdateLegacy = 0,
			kUpdateWorld = 1,
			kUpdateBody = 2,
		};
		// 	def skew(self,x): #skew-symmetric
		// 	    return np.array([[0, -x[2], x[1]],
		// 	                     [x[2], 0, -x[0]],
		// 	                     [-x[1], x[0], 0]])
		Eigen::Matrix3f skew(Eigen::Vector3f x){
			Eigen::Matrix3f skew;
			skew << 0, -x[2], x[1],
			     x[2], 0, -x[0],
			     -x[1], x[0], 0;
	        //print_eigen(skew)
	        return skew;
	    }
	
		Eigen::Matrix3f exp_map(Eigen::Vector3f phi){
			// print_string("exp_map<1>");
			Eigen::Matrix3f skew_phi_matrix=this->skew(phi);
			Eigen::Matrix3f I=Eigen::Matrix<float, 3, 3>::Identity();
			// print_eigen(I);
			float phi_norm =phi.norm();
			float sin_phi_norm_div_phi_norm=sin(phi_norm)/phi_norm;	
			// print_string("exp_map<1.5>");
			float one_minus_cos_phi_norm_div_phi_norm_square=(1.0-cos(phi_norm))/(phi_norm*phi_norm);
			// print_string("exp_map<1.6>");
			Eigen::Matrix3f skew_phi_matrix_square=skew_phi_matrix*skew_phi_matrix;
			Eigen::Matrix3f exp_map=I+sin_phi_norm_div_phi_norm*skew_phi_matrix+one_minus_cos_phi_norm_div_phi_norm_square*skew_phi_matrix_square;
			// print_string("exp_map<2>");
			return exp_map;
	    }	

		FovOptimizerOnManifold(std::string filename_, double visibility_ ,double visibility_angle_,bool importing_, std::vector<Eigen::Vector3f> points_list_, Eigen::Vector3f starting_c_,bool print,Eigen::Vector3f ref_point, int cnt)
	    {
	    this->brute_force_max_feature_in_FOV=0;
	    this->brute_force_max_visibility=0.0f;
		this->print_path=print;
		this->no_io = no_io_enabled();
		this->bf_coarse_to_fine = fov_env_flag("FOV_BF_COARSE_TO_FINE", false);
		this->bf_require_global = fov_env_flag("FOV_BF_REQUIRE_GLOBAL", true);
		this->bf_coarse_stride = fov_env_int("FOV_BF_COARSE_STRIDE", 4, 1, 1000000);
		this->bf_refine_deg = fov_env_float("FOV_BF_REFINE_DEG", 10.0f, 0.1f, 180.0f);
		if (this->no_io) {
			this->print_path = false;
		}
		const std::string dp = fov_monte_carlo_data_prefix();
		const std::string pathfile_path = "../../Data/" + dp + "path.csv";
		const std::string pointsfile_path = "../../Data/" + dp + "points.csv";
		const std::string quivers_path = "../../Data/" + dp + "quiversforonepoint.csv";
		const std::string ajl_path = "../../Data/" + dp + "ajl.csv";
		const std::string error_curve_path = "../../Data/" + dp + "errorcurve.csv";
		const std::string j_curve_path = "../../Data/" + dp + "Jcurve.csv";
		const std::string quaternions_path = "../../Data/" + dp + "quaternions.csv";
		const std::string r_updates_path = "../../Data/" + dp + "Rwhileopt.csv";

		if (!this->no_io) {
			this->pathfile.open(pathfile_path);
		    this->pointsfile.open(pointsfile_path);
			/*ADDED:*/
			this->quiversforonepoint.open(quivers_path, std::ios::app);
			this->ajl.open(ajl_path, std::ios::app);
			this->error_curve.open(error_curve_path, std::ios::app);
			this->J_curve.open(j_curve_path, std::ios::app);
			this->quaternions.open(quaternions_path, std::ios::app);
			this->R_updates_for_ti.open(r_updates_path, std::ios::app);

			write_header_if_empty(pathfile_path, this->pathfile, "rot_x,rot_y,rot_z");
			write_header_if_empty(pointsfile_path, this->pointsfile, "x,y,z");
			write_header_if_empty(quivers_path, this->quiversforonepoint,
			                      "id,ref_x,ref_y,ref_z,dir_x,dir_y,dir_z,feature_count,degree_between,j_norm,step,visibility");
			write_header_if_empty(ajl_path, this->ajl, "ajl_x,ajl_y,ajl_z");
			write_header_if_empty(error_curve_path, this->error_curve, "visibility");
			write_header_if_empty(j_curve_path, this->J_curve, "j_norm");
			write_header_if_empty(quaternions_path, this->quaternions, "qw,qx,qy,qz");
			write_header_if_empty(r_updates_path, this->R_updates_for_ti, "rx,ry,rz");
		}
		this->ref_point = ref_point;
	    this->R= Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitX())
			  * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitY())
			  * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitZ());
		/*ADDED:*/
		this->cnt=cnt;
  		this->c=starting_c_/starting_c_.norm();
	    this->rotated_vec=this->R*this->c;  /*0 * starting_c*/
 		if (importing_==true){
 			this->points_list=points_list_;
 		}

	    else{
	    	for (int i=0;i< 5 ;i++){   /*200*/
	    		Eigen::Vector3f a;
	    		Eigen::Vector3f b;
	    		a<<rand() % 10+1+20,rand() % 20+1+50,rand() % 20+1+0;
	    		b<<rand() % 10+1-30,rand() % 10+1+10,rand() %50+1+50;
	    		//print_eigen_v(a);
	    		//print_eigen_v(b);
	    		this->points_list.push_back(a); //thisrand() % 10+1
	    		this->points_list.push_back(b);

	    	} 
	    }
	    this->t<<0,0,0;

	    for(Eigen::Vector3f point: this->points_list){
	    	Eigen::Vector3f K__=point-this->t;
	    	K__=K__/K__.norm();		    
	    	this->K_list.push_back(K__);
    		std::ostringstream ss;
			ss << K__[0]<<","<< K__[1]<<","<< K__[2]<<std::endl;
			std::string s(ss.str());
			if (!this->no_io) {
				this->pointsfile<<s;
			}
	    }

	    this->filename=filename_;
	    this->visibility_angle=visibility_angle_;
	    this->optimize_visibility_sigmoid=visibility_;
	    this->optimize_visibility=!visibility_;

	   	this->loader=new CloudLoader;
		this->loader->ImportFromXyzFile("../../brute_force_xyz_indexes_two_degree.csv",1,true,false,",");
		// std::cout<<"biiiiiiiiib";
		this->loaderrot=new CloudLoader;
		this->loaderrot->ImportFromXyzFile("../../all_rotation.csv",1,true,false,",");
	}	

		void set_update_frame_mode(UpdateFrameMode mode) {
			this->update_frame_mode = mode;
		}
		void set_step_clamp_enabled(bool enabled) {
			this->step_clamp_enabled = enabled;
		}
		void set_step_limits_deg(float min_deg, float max_deg) {
			if (min_deg > 0.0f) {
				this->min_step_rad = min_deg * static_cast<float>(M_PI) / 180.0f;
			}
			if (max_deg > 0.0f) {
				this->max_step_rad = max_deg * static_cast<float>(M_PI) / 180.0f;
			}
		}
		void set_use_fov_schedule(bool enabled) {
			this->use_fov_schedule = enabled;
		}
		void set_use_ks_schedule(bool enabled) {
			this->use_ks_schedule = enabled;
		}
		void set_ks_transition_deg(float value) {
			if (value > 0.0f) {
				this->ks_transition_deg = value;
			}
		}
		void set_fov_schedule_deg(const std::vector<float>& schedule_deg) {
			this->fov_schedule_rad.clear();
			for (float value : schedule_deg) {
				if (value > 0.0f) {
					this->fov_schedule_rad.push_back(
						value * static_cast<float>(M_PI) / 180.0f);
				}
			}
			this->use_fov_schedule = true;
		}
		bool is_no_io() const {
			return this->no_io;
		}

	    Eigen::Matrix3f get_R(void){
	   		return this->R;
	   }

	    Eigen::Vector3f get_brute_force_best_vector_feat(void){
	   		return this->brute_force_best_vector_w_feature;
	    }
	    
		Eigen::Vector3f get_brute_force_best_vector_vis(void){
			return this->brute_force_best_vector_w_vis;
		}

	    int get_brute_force_max_feature(){
	   		return this->brute_force_max_feature_in_FOV;
	    }

		float get_brute_force_max_visibility(){
			return this->brute_force_max_visibility;
		}

	    int get_optimized_max_feature(){
			int optimized_max_feature_in_FOV_=0;
	   		Eigen::Vector3f optimized_FOV_vector=this->R*this->c;;
			for(Eigen::Vector3f K_: this->K_list){
				if( abs(acos((K_.transpose())*optimized_FOV_vector))<this->visibility_alpha){
					optimized_max_feature_in_FOV_+=1;
				}
			}

			this->optimized_max_feature_in_FOV=optimized_max_feature_in_FOV_;
	   		return this->optimized_max_feature_in_FOV;
	    }

	    /*ADDED:*/
		float get_optimized_max_visibility(){
			float max_visibility_in_fov = 0.0f;
			for(Eigen::Vector3f K_: this->K_list){
				float cos_theta = K_.transpose() * this->R * this->c;
				float visibility = 1.0f / (1.0f + std::exp(-ks * (cos_theta - std::cos(this->visibility_alpha))));
				max_visibility_in_fov +=  visibility;
			} 
			return max_visibility_in_fov;
   	    }

		float ScoreVisibilityDirection(const Eigen::Vector3f& direction) const {
			if (direction.norm() < 1e-6f) {
				return 0.0f;
			}
			const Eigen::Vector3f unit_direction = direction / direction.norm();
			float visibility_sum = 0.0f;
			for (const Eigen::Vector3f& K_ : this->K_list) {
				const float cos_theta = K_.transpose() * unit_direction;
				const float visibility =
				    1.0f / (1.0f + std::exp(-ks * (cos_theta - std::cos(this->visibility_alpha))));
				visibility_sum += visibility;
			}
			return visibility_sum;
		}

		/*ADDED:*/
		int get_count_in_fov(){
			int feature_in_FOV_=0;
			Eigen::Vector3f optimized_FOV_vector=this->R*this->c;
			for(Eigen::Vector3f K_: this->K_list){
				if( abs(acos((K_.transpose())*optimized_FOV_vector))<this->visibility_alpha){
					feature_in_FOV_+=1;
				}
			}
			return feature_in_FOV_;
		}
		
		/*ADDED:*/
		void brute_force_search_with_feature_count(void){
			const std::vector<Eigen::Vector3f>& directions = this->loader->get_pointcloud();
			if (directions.empty()) {
				return;
			}

			auto eval_count = [&](const Eigen::Vector3f& direction) -> int {
				int max_feature_in_FOV = 0;
				for (Eigen::Vector3f K_ : this->K_list) {
					if (std::abs(std::acos((K_.transpose()) * direction)) < this->visibility_alpha) {
						max_feature_in_FOV += 1;
					}
				}
				return max_feature_in_FOV;
			};

			int best_count = this->brute_force_max_feature_in_FOV;
			Eigen::Vector3f best_dir = this->brute_force_best_vector_w_feature;
			if (best_dir.norm() < 1e-6f) {
				best_dir = directions.front();
				if (best_dir.norm() > 1e-6f) {
					best_dir = best_dir / best_dir.norm();
				}
			}
			auto try_update = [&](const Eigen::Vector3f& direction, int count) {
				if (count > best_count) {
					best_count = count;
					best_dir = direction;
				}
			};

			if (this->bf_coarse_to_fine) {
				const int stride = std::max(1, this->bf_coarse_stride);
				for (size_t i = 0; i < directions.size(); i += static_cast<size_t>(stride)) {
					Eigen::Vector3f direction = directions[i];
					if (direction.norm() < 1e-6f) {
						continue;
					}
					direction = direction / direction.norm();
					try_update(direction, eval_count(direction));
				}

				if (!this->bf_require_global) {
					const float cos_refine = std::cos(this->bf_refine_deg * static_cast<float>(M_PI) / 180.0f);
					for (const Eigen::Vector3f& raw_direction : directions) {
						Eigen::Vector3f direction = raw_direction;
						if (direction.norm() < 1e-6f) {
							continue;
						}
						direction = direction / direction.norm();
						if (direction.dot(best_dir) < cos_refine) {
							continue;
						}
						try_update(direction, eval_count(direction));
					}
					this->brute_force_max_feature_in_FOV = best_count;
					this->brute_force_best_vector_w_feature = best_dir;
					return;
				}
			}

			for (const Eigen::Vector3f& raw_direction : directions) {
				Eigen::Vector3f direction = raw_direction;
				if (direction.norm() < 1e-6f) {
					continue;
				}
				direction = direction / direction.norm();
				try_update(direction, eval_count(direction));
			}

			this->brute_force_max_feature_in_FOV = best_count;
			this->brute_force_best_vector_w_feature = best_dir;
			return;

			// NOTE:NOTE: commentedddddddddddddddddddddd
			// this->c=this->brute_force_best_vector;
		    // this->R= Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitX())
			// 	  * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitY())
			// 	  * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitZ());
			// this->optimize(this->brute_force_best_vector);

			// Eigen::Vector3f best_updated_brute_force_vector=this->R*this->c;
			// int new_max_feature_in_FOV=0;
			// for(Eigen::Vector3f K_: this->K_list){

			// 	if( abs(acos((K_.transpose())*best_updated_brute_force_vector))<this->visibility_alpha){
			// 		new_max_feature_in_FOV+=1;
			// 	}
			// }

			// if (new_max_feature_in_FOV>=this->brute_force_max_feature_in_FOV){
			// 	this->brute_force_max_feature_in_FOV=new_max_feature_in_FOV;
			// 	this->brute_force_best_vector=best_updated_brute_force_vector;
			// 	std::cout<<"updated brute_force_max_feature_in_FOV "<<this->brute_force_max_feature_in_FOV<<std::endl;
			// }

	   		// print_string("degree_difference");
	   		// // print_eigen_v(this->brute_force_best_vector);
	   		// print_float(acos(optimized_FOV_vector.transpose()*this->brute_force_best_vector)*180.0/M_PI);
	   }
   		
	   /*ADDED:*/
	    void brute_force_search_with_visibility(void){
			const std::vector<Eigen::Vector3f>& directions = this->loader->get_pointcloud();
			if (directions.empty()) {
				return;
			}

			auto eval_visibility = [&](const Eigen::Vector3f& direction) -> float {
				float max_visibility_in_fov = 0.0f;
				for (Eigen::Vector3f K_ : this->K_list) {
					float cos_theta = K_.transpose() * direction;
					float visibility = 1.0f / (1.0f + std::exp(-ks * (cos_theta - std::cos(this->visibility_alpha))));
					max_visibility_in_fov += visibility;
				}
				return max_visibility_in_fov;
			};

			float best_visibility = this->brute_force_max_visibility;
			Eigen::Vector3f best_dir = this->brute_force_best_vector_w_vis;
			if (best_dir.norm() < 1e-6f) {
				best_dir = directions.front();
				if (best_dir.norm() > 1e-6f) {
					best_dir = best_dir / best_dir.norm();
				}
			}
			auto try_update = [&](const Eigen::Vector3f& direction, float score) {
				if (score > best_visibility) {
					best_visibility = score;
					best_dir = direction;
				}
			};

			if (this->bf_coarse_to_fine) {
				const int stride = std::max(1, this->bf_coarse_stride);
				for (size_t i = 0; i < directions.size(); i += static_cast<size_t>(stride)) {
					Eigen::Vector3f direction = directions[i];
					if (direction.norm() < 1e-6f) {
						continue;
					}
					direction = direction / direction.norm();
					try_update(direction, eval_visibility(direction));
				}

				if (!this->bf_require_global) {
					const float cos_refine = std::cos(this->bf_refine_deg * static_cast<float>(M_PI) / 180.0f);
					for (const Eigen::Vector3f& raw_direction : directions) {
						Eigen::Vector3f direction = raw_direction;
						if (direction.norm() < 1e-6f) {
							continue;
						}
						direction = direction / direction.norm();
						if (direction.dot(best_dir) < cos_refine) {
							continue;
						}
						try_update(direction, eval_visibility(direction));
					}
					this->brute_force_max_visibility = best_visibility;
					this->brute_force_best_vector_w_vis = best_dir;
					optimized_max_vis = get_optimized_max_visibility();
					if (optimized_max_vis > this->brute_force_max_visibility) {
						this->brute_force_max_visibility = optimized_max_vis;
						this->brute_force_best_vector_w_vis = this->R * this->c;
					}
					return;
				}
			}

			for (const Eigen::Vector3f& raw_direction : directions) {
				Eigen::Vector3f direction = raw_direction;
				if (direction.norm() < 1e-6f) {
					continue;
				}
				direction = direction / direction.norm();
				try_update(direction, eval_visibility(direction));
			}

			this->brute_force_max_visibility = best_visibility;
			this->brute_force_best_vector_w_vis = best_dir;
			optimized_max_vis = get_optimized_max_visibility();
			if (optimized_max_vis > this->brute_force_max_visibility) {
				this->brute_force_max_visibility = optimized_max_vis;
				this->brute_force_best_vector_w_vis = this->R * this->c;
			}
			return;
		}

	    Eigen::Vector3f get_Jacobian_from_K_and_C(float K1,float K2,float K3,float C1,float C2,float C3){ // formula 6
	   			Eigen::Vector3f J;
	   			float a=C2*K3-C3*K2;
	   			float b=C3*K1-C1*K3;
	   			float c=C1*K2-C2*K1;
	   			J<<a,b,c;
	   			return J;	   	
	   }

		int optimize(Eigen::Vector3f brute_force) {
			int redo = 0;
			const float points_count =
				std::max(1.0f, static_cast<float>(this->K_list.size()));
			const float inv_points = 1.0f / points_count;
			const float max_step_rad = this->max_step_rad;
			const float rel_tol = 1e-4f;
			const float grad_tol = 1e-4f;
			const int patience = 3;
			int no_improve_count = 0;

			float degree_between;
			int feature_count;
			float step = 0.0f;
			float v = 0.0f;
			float w = 0.0f;
			float coeff = 0.0f;
			float prev_visibility = std::numeric_limits<float>::quiet_NaN();
			const bool log_io = !this->no_io;
			// std::cout<<"starting c"<<this->c<<std::endl;
			for (int i = 0; i < this->max_iteration; i++) {
				Eigen::Vector3f aJ_l;
				aJ_l << 0, 0, 0;
				float residual = 0.0;
				float current_visibility = 0.0f;
				const float visibility_alpha = GetVisibilityAlpha(i);
				const float cos_alpha = std::cos(visibility_alpha);
				//++++++++++++++++++++++++++++++++++++++++++++++++   iterating on landmarks  ++++++++++++++++++++++++++++++++++++++++++++++++
				for (Eigen::Vector3f K_ : this->K_list) {
					Eigen::Vector3f K = (K_.transpose())* this->R; 
					float C1 = this->c[0];
					float C2 = this->c[1];
					float C3 = this->c[2];
					float K1 = K[0];
					float K2 = K[1];
					float K3 = K[2];
		
					Eigen::Vector3f F_Jacobian = this->get_Jacobian_from_K_and_C(K1, K2, K3, C1, C2, C3);
					Eigen::Vector3f J = F_Jacobian;
		
					if (this->optimize_visibility == true) {
						float KTRC = K_.transpose() * this->R * this->c; 
						residual = residual + KTRC;
						F_Jacobian = 2 * 0.35 * KTRC * J + 0.11 * J;   
					}
		
					if (this->optimize_visibility_sigmoid == true) {
						float u, KTRC;
						u = (K_.transpose()) * (this->R) * this->c;  
						KTRC = u;
						residual = residual + KTRC;
						const float ks_iter = static_cast<float>(GetKsForIteration(i));

						w = (-1.0f) * ks_iter * (u - cos_alpha); 
						v = exp(w);
						coeff = (-1.0f) * (pow((1 + v), (-2)) * v * (0.0f - ks_iter)); 
						F_Jacobian = coeff * J; 
						current_visibility += 1.0f / (1.0f + v);
					} else {
						const float u = (K_.transpose()) * (this->R) * this->c;
						current_visibility += (u >= cos_alpha) ? 1.0f : 0.0f;
					}
					Eigen::Vector3f aJ = F_Jacobian;
					aJ_l = aJ_l + aJ;
				} 
				//++++++++++++++++++++++++++++++++++++++++++++++++++  Done iterating on landmarks ++++++++++++++++++++++++++++++++++++++++++++++++

				aJ_l *= inv_points;
				current_visibility *= inv_points;

				const float jacobian_norm = aJ_l.norm();
				if (jacobian_norm > 1e-9f) {
					step = 1.0f;
					if (this->step_clamp_enabled) {
						float delta_norm = step * jacobian_norm;
						if (delta_norm > max_step_rad) {
							step = max_step_rad / jacobian_norm;
						}
					}
				} else {
					step = 0.0f;
				}

				// std::cout<<aJ_l<<std::endl;
				/*______________________________________________________________________________________-logging_________________*/
				if (log_io) {
					this->ajl << aJ_l << std::endl;
					feature_count = this->get_count_in_fov();
					degree_between = acos(brute_force.transpose() * rotated_vec) * 180.0 / M_PI;
					this->quiversforonepoint << this->cnt << "," << std::to_string(this->ref_point[0]) << "," 
											 << std::to_string(this->ref_point[1]) << "," << std::to_string(this->ref_point[2]) << ","
											 << std::to_string(this->rotated_vec[0]) << "," << std::to_string(this->rotated_vec[1]) << "," 
											 << std::to_string(this->rotated_vec[2]) << "," << feature_count << "," << degree_between << ","
											 << jacobian_norm << ","<< step <<"," << current_visibility<< std::endl;
				}


				if (std::isfinite(prev_visibility)) {
					const float denom =
						std::max(1.0f, std::abs(prev_visibility));
					const float rel_improve =
						std::abs(current_visibility - prev_visibility) / denom;
					if (rel_improve < rel_tol || jacobian_norm < grad_tol) {
						no_improve_count++;
					} else {
						no_improve_count = 0;
					}
				}
				prev_visibility = current_visibility;
				if (no_improve_count >= patience) {
					break;
				}

/*___________________________________________ ________________________________________-update R__________________*/
				const Eigen::Vector3f delta_body = step * aJ_l;
				if (delta_body.norm() > 1e-9f) {
					ApplyRotationUpdate(delta_body);
				}
				this->rotated_vec = this->R * this->c;
				// Eigen::Quaternionf quat = matrixToQuaternion(this->R);
/*_____________________________________________________________________________________-logging_________________*/

				// this->quaternions<<quat.w()<< "," << quat.x() << ","<< quat.y() << "," << quat.z() << std::endl;
				if (log_io) {
					this->R_updates_for_ti << this->R * this->c << std::endl;
					if (this->print_path == true) {
						this->pathfile << std::to_string(rotated_vec[0]) << "," << std::to_string(rotated_vec[1]) << "," 
									   << std::to_string(rotated_vec[2]) << std::endl;
					}
				}
			}
		
			if (log_io) {
				this->R_updates_for_ti << "\n\n";
				this->ajl << "\n\n";
			}
			return redo;
		}

		// Trajectory-style visibility-only optimizer with scheduled alpha.
		// Call this overload with `true` to avoid changing existing behavior.
		int optimize(Eigen::Vector3f brute_force, bool use_scheduled_alpha) {
			if (!use_scheduled_alpha) {
				return this->optimize(brute_force);
			}

			int redo = 0;
			const float points_count =
				std::max(1.0f, static_cast<float>(this->K_list.size()));
			const float inv_points = 1.0f / points_count;
			const float rel_tol = 1e-4f;
			const float grad_tol = 1e-4f;
			const int patience = 3;
			int no_improve_count = 0;
			float prev_avg_jac_norm = 1.0f;
			float adaptive_step_scale = 1.0f;
			const bool step_norm_jacobian = false;  // match kStepNormPoints
			const bool adaptive_step_enabled = true;
			const float base_step_scale = 1.0f;
			const float max_step_rad = this->max_step_rad;
			const float max_step_decay_strength = 0.5f;
			const float adapt_max_clip_thresh = 0.35f;
			const float adapt_low_max_clip_thresh = 0.05f;
			const float adapt_shrink_factor = 0.70f;
			const float adapt_scale_min = 0.10f;
			const float adapt_scale_max = 10.0f;
			const bool log_io = !this->no_io;

			const float alpha_260 = 260.0f * static_cast<float>(M_PI) / 180.0f;
			const float schedule_rad[] = {
				alpha_260,
				static_cast<float>(this->visibility_alpha_180),
				static_cast<float>(this->visibility_alpha_90),
				static_cast<float>(this->visibility_alpha)
			};
			const int schedule_len = 4;

			float degree_between = 0.0f;
			int feature_count = 0;
			float step = 0.0f;
			float v = 0.0f;
			float w = 0.0f;
			float coeff = 0.0f;
			float prev_visibility = std::numeric_limits<float>::quiet_NaN();

			for (int i = 0; i < this->max_iteration; i++) {
				Eigen::Vector3f aJ_l;
				aJ_l << 0, 0, 0;
				float current_visibility = 0.0f;

				float visibility_alpha = static_cast<float>(this->visibility_alpha);
				if (this->use_fov_schedule) {
					if (!this->fov_schedule_rad.empty()) {
						visibility_alpha = GetVisibilityAlpha(i);
					} else {
						const int stage_len = std::max(1, this->max_iteration / schedule_len);
						int stage_idx = i / stage_len;
						if (stage_idx >= schedule_len) {
							stage_idx = schedule_len - 1;
						}
						visibility_alpha = schedule_rad[stage_idx];
					}
				}
				const float cos_alpha = std::cos(visibility_alpha);
				float ks_iter = static_cast<float>(this->ks);
				if (this->use_ks_schedule && this->ks_transition_deg > 0.0f) {
					const double sin_alpha = std::sin(static_cast<double>(visibility_alpha));
					const double delta_rad =
						static_cast<double>(this->ks_transition_deg) * M_PI / 180.0;
					if (std::abs(sin_alpha) > 1e-6 && delta_rad > 0.0) {
						const double dyn = 2.94 / (delta_rad * sin_alpha);
						if (std::isfinite(dyn) && dyn > 0.0) {
							ks_iter = static_cast<float>(dyn);
						}
					}
				}

				for (Eigen::Vector3f K_ : this->K_list) {
					Eigen::Vector3f K = (K_.transpose()) * this->R;
					float C1 = this->c[0];
					float C2 = this->c[1];
					float C3 = this->c[2];
					float K1 = K[0];
					float K2 = K[1];
					float K3 = K[2];

					Eigen::Vector3f F_Jacobian =
						this->get_Jacobian_from_K_and_C(K1, K2, K3, C1, C2, C3);
					Eigen::Vector3f J = F_Jacobian;

					if (this->optimize_visibility_sigmoid) {
						const float u = (K_.transpose()) * (this->R) * this->c;
						w = (-1.0f) * ks_iter * (u - cos_alpha);
						v = std::exp(w);
						coeff =
							(-1.0f) *
							(std::pow((1.0f + v), (-2.0f)) * v * (0.0f - ks_iter));
						F_Jacobian = coeff * J;
						current_visibility += 1.0f / (1.0f + v);
					} else {
						const float u = (K_.transpose()) * (this->R) * this->c;
						current_visibility += (u >= cos_alpha) ? 1.0f : 0.0f;
					}

					aJ_l = aJ_l + F_Jacobian;
				}

				aJ_l *= inv_points;
				current_visibility *= inv_points;

				const float jacobian_norm = aJ_l.norm();
				const float iter_ratio =
					static_cast<float>(i) / std::max(1, this->max_iteration - 1);
				const float max_step_iter =
					std::max(0.0f, max_step_rad * (1.0f - max_step_decay_strength * iter_ratio));
				const float norm_scale =
					step_norm_jacobian
						? std::max(1e-6f, prev_avg_jac_norm)
						: 1.0f;

				float base_step = 0.0f;
				float target_delta_norm = 0.0f;
				bool clipped_max = false;
				if (jacobian_norm > 1e-9f) {
					base_step = (adaptive_step_scale * base_step_scale) / norm_scale;
					target_delta_norm = base_step * jacobian_norm;
					if (this->step_clamp_enabled) {
						if (target_delta_norm > max_step_iter) {
							target_delta_norm = max_step_iter;
							clipped_max = true;
						}
					}
					step = target_delta_norm / jacobian_norm;
					const Eigen::Vector3f delta_body = step * aJ_l;
					if (delta_body.norm() > 1e-9f) {
						ApplyRotationUpdate(delta_body);
					}
				} else {
					step = 0.0f;
				}

				this->rotated_vec = this->R * this->c;

				if (log_io) {
					this->ajl << aJ_l << std::endl;
					feature_count = this->get_count_in_fov();
					degree_between =
						acos(brute_force.transpose() * rotated_vec) * 180.0f / M_PI;
					this->quiversforonepoint << this->cnt << ","
					                         << std::to_string(this->ref_point[0]) << ","
					                         << std::to_string(this->ref_point[1]) << ","
					                         << std::to_string(this->ref_point[2]) << ","
					                         << std::to_string(this->rotated_vec[0]) << ","
					                         << std::to_string(this->rotated_vec[1]) << ","
					                         << std::to_string(this->rotated_vec[2]) << ","
					                         << feature_count << "," << degree_between << ","
					                         << aJ_l.norm() << "," << step << ","
					                         << current_visibility << std::endl;

					this->R_updates_for_ti << this->R * this->c << std::endl;
					if (this->print_path == true) {
						this->pathfile << std::to_string(rotated_vec[0]) << ","
						               << std::to_string(rotated_vec[1]) << ","
						               << std::to_string(rotated_vec[2]) << std::endl;
					}
				}

				if (step_norm_jacobian && jacobian_norm > 1e-9f) {
					prev_avg_jac_norm = jacobian_norm;
				}

				if (adaptive_step_enabled && jacobian_norm > 1e-9f) {
					const float max_clip_ratio = clipped_max ? 1.0f : 0.0f;
					if (max_clip_ratio > adapt_max_clip_thresh) {
						adaptive_step_scale = std::max(
							adapt_scale_min, adaptive_step_scale * adapt_shrink_factor);
					} else if (max_clip_ratio < adapt_low_max_clip_thresh) {
						adaptive_step_scale = std::min(
							adapt_scale_max,
							adaptive_step_scale * 1.05f);
					}
				}

				if (std::isfinite(prev_visibility)) {
					const float denom =
						std::max(1.0f, std::abs(prev_visibility));
					const float rel_improve =
						std::abs(current_visibility - prev_visibility) / denom;
					if (rel_improve < rel_tol || jacobian_norm < grad_tol) {
						no_improve_count++;
					} else {
						no_improve_count = 0;
					}
				}
				prev_visibility = current_visibility;
				if (no_improve_count >= patience) {
					break;
				}
			}

			if (log_io) {
				this->R_updates_for_ti << "\n\n";
				this->ajl << "\n\n";
			}
			return redo;
		}
		
		/*ADDED: for steepest desent visual*/
		void calculate_error_all_direction_for_each_t() {
			if (this->no_io) {
				return;
			}
			float error = 0.0f;
			for(Eigen::Vector3f direction:this->loader->get_pointcloud()){	
				direction=direction/direction.norm();
				error=calculate_error_for_some_r(direction); 
				this->error_curve <<error<<std::endl; 
       		}
		}

		float calculate_error_for_some_r(Eigen::Vector3f& direction) {
			// std::cout<<"error calculator function from"<<this->cnt<<std::endl;
			float error = 0.0f;
			for(Eigen::Vector3f K_: this->K_list){
				float cos_theta = K_.transpose() * direction;
		        float visibility = 1.0f / (1.0f + std::exp(-ks * (cos_theta - std::cos(this->visibility_alpha))));
				error +=  visibility;
			} //accumulated error for direction dir
			return error;
		}

		Eigen::Matrix3f direction_to_rotation(Eigen::Vector3f& a, Eigen::Vector3f& b) {
			Eigen::Vector3f a_norm = a/a.norm();
			Eigen::Vector3f b_norm = b/b.norm();
			
			// Handle special cases
			if(a_norm.isApprox(b_norm)) {
				return Eigen::Matrix3f::Identity();
			}
			if(a_norm.isApprox(-b_norm)) {
				// Find orthogonal vector for 180 degree rotation
				Eigen::Vector3f orth = (std::abs(a_norm[0]) < 0.9) ? Eigen::Vector3f(1, 0, 0) : Eigen::Vector3f(0, 1, 0);
				Eigen::Vector3f axis = a_norm.cross(orth).normalized();
				return Eigen::AngleAxisf(M_PI, axis).toRotationMatrix();
			}
			Eigen::Vector3f axis = a_norm.cross(b_norm).normalized();
			double angle = acos(a_norm.dot(b_norm));	
			Eigen::Matrix3f R = Eigen::AngleAxisf(angle, axis).toRotationMatrix();
			return R;
		}	

		void calculate_J_all_direction_for_each_t() {
			if (this->no_io) {
				return;
			}
			for(Eigen::Vector3f direction:this->loader->get_pointcloud()){	
				Eigen::Vector3f aJ_l;
				aJ_l << 0, 0, 0;
				Eigen::Matrix3f r= direction_to_rotation(this->c, direction);
				// std::cout<<"direction  "<<direction<<" rc  "<<r*this->c<<std::endl; checked
				for(Eigen::Vector3f K_: this->K_list){
					Eigen::Vector3f K = (K_.transpose()) * r;
					float C1 = this->c[0];
					float C2 = this->c[1];
					float C3 = this->c[2];
					float K1 = K[0];
					float K2 = K[1];
					float K3 = K[2];
		
					Eigen::Vector3f F_Jacobian = this->get_Jacobian_from_K_and_C(K1, K2, K3, C1, C2, C3);
					Eigen::Vector3f J = F_Jacobian;
					float u, KTRC;
					u = (K_.transpose())*r*this->c;  
					KTRC = u;		
					float w = (-1.0) * this->ks * (u - cos(this->visibility_alpha)); 
					float v = exp(w);
					float coeff = (-1.0) * (pow((1 + v), (-2)) * v * (0.0 - this->ks)); 
					F_Jacobian = coeff * J; 
					Eigen::Vector3f aJ = F_Jacobian;
					aJ_l = aJ_l + aJ;
				} 
				this->J_curve <<aJ_l.norm()<<std::endl; 
       		}
		}

private:
		int max_iteration=30;
		Eigen::Matrix3f R;
		Eigen::Vector3f c;
		Eigen::Vector3f ref_point;

		std::vector<Eigen::Vector3f> points_list;
		Eigen::Vector3f t;
		std::vector<Eigen::Vector3f> K_list;
		std::vector<double> plotR1;
		std::vector<double> plotR2;
		std::vector<double> plotR3;
		std::vector<double> plot_sum_x;
		std::vector<double> plot_sum_y;	

		bool optimize_visibility=false;
		bool optimize_visibility_sigmoid;
		std::string filename;
		double ks=15;
			float visibility_angle=15.0;
		double visibility_alpha=visibility_angle*M_PI/180.0;
		double visibility_alpha_180=179.0*M_PI/180.0;
		double visibility_alpha_120=120.0*M_PI/180.0;
		double visibility_alpha_90=90.0*M_PI/180.0;
		double visibility_alpha_45=45.0*M_PI/180.0;
		double visibility_alpha_22_5=22.5*M_PI/180.0;
		bool use_fov_schedule=false;
		bool use_ks_schedule=false;
		float ks_transition_deg=0.0f;
		std::vector<float> fov_schedule_rad;
		int update_frame_mode=kUpdateWorld;
		bool step_clamp_enabled=true;
		float min_step_rad=0.25f * static_cast<float>(M_PI) / 180.0f;
		float max_step_rad=5.0f * static_cast<float>(M_PI) / 180.0f;
		float optimized_max_vis;

		int cnt;
		std::vector<double> plotstep;                                              
		Eigen::Vector3f rotated_vec;
		std::ofstream pointsfile;
		std::ofstream pathfile;
		std::ifstream brute_force_file;
		std::ofstream quiversforonepoint;
		std::ofstream ajl;
		std::ofstream error_curve;
		std::ofstream J_curve;
		std::ofstream quaternions;
		std::ofstream R_updates_for_ti;
		bool print_path;
		bool no_io = false;
		bool bf_coarse_to_fine = false;
		bool bf_require_global = true;
		int bf_coarse_stride = 4;
		float bf_refine_deg = 10.0f;
		CloudLoader *loader;
		CloudLoader *loaderrot;
		int brute_force_max_feature_in_FOV;
		float brute_force_max_visibility;
		int optimized_max_feature_in_FOV;
		Eigen::Vector3f brute_force_best_vector_w_vis;
		Eigen::Vector3f brute_force_best_vector_w_feature;

		float GetVisibilityAlpha(int iteration_count) const {
			if (!this->use_fov_schedule) {
				return static_cast<float>(this->visibility_alpha);
			}
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
			if (ratio < 0.2f) {
				return static_cast<float>(this->visibility_alpha_180);
			}
			if (ratio < 0.3f) {
				return static_cast<float>(this->visibility_alpha_120);
			}
			if (ratio < 0.4f) {
				return static_cast<float>(this->visibility_alpha_90);
			}
			return static_cast<float>(this->visibility_alpha);
		}

		double GetKsForIteration(int iteration_count) const {
			if (!this->use_ks_schedule || this->ks_transition_deg <= 0.0f) {
				return this->ks;
			}
			const float alpha = GetVisibilityAlpha(iteration_count);
			const double sin_alpha = std::sin(static_cast<double>(alpha));
			if (std::abs(sin_alpha) < 1e-6) {
				return this->ks;
			}
			const double delta_rad =
				static_cast<double>(this->ks_transition_deg) * M_PI / 180.0;
			if (delta_rad <= 0.0) {
				return this->ks;
			}
			const double dyn = 2.94 / (delta_rad * sin_alpha);
			if (!std::isfinite(dyn) || dyn <= 0.0) {
				return this->ks;
			}
			return dyn;
		}

		void ApplyRotationUpdate(const Eigen::Vector3f& delta_body) {
			if (this->update_frame_mode == kUpdateBody) {
				this->R = this->R * this->exp_map(delta_body);
			} else if (this->update_frame_mode == kUpdateWorld) {
				const Eigen::Vector3f delta_world = this->R * delta_body;
				this->R = this->exp_map(delta_world) * this->R;
			} else {
				this->R = this->exp_map(delta_body) * this->R;
			}
		}

};

#endif // USE_MANIFOLD_BACKUP
#endif // _MANIFOLD_
