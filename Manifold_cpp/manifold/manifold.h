// Checks if _ANIMALS IF DECLARED
#ifndef _MANIFOLD_
  
// Defines _ANIMALS_ if above
// conditions fails
#define _MANIFOLD_


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
// class Manifold_minimize_angle_among_all_target_points(object):

class FovOptimizerOnManifold{
public:
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
		this->pathfile.open ("path.csv");
	    this->pointsfile.open ("points.csv");
/*ADDED:*/
		this->quiversforonepoint.open("../../Data/quiversforonepoint.csv", std::ios::app);
		this->ref_point = ref_point;
		this->ajl.open("../../Data/ajl.csv", std::ios::app);
		this->error_curve.open("../../Data/errorcurve.csv", std::ios::app);
		this->J_curve.open("../../Data/Jcurve.csv", std::ios::app);
		this->quaternions.open("../../Data/quaternions.csv", std::ios::app);
		this->R_updates_for_ti.open("../../Data/Rwhileopt.csv", std::ios::app);
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
    		this->pointsfile<<s;
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
			double visibility_limit = 22.5*M_PI/180.0;
			for(Eigen::Vector3f K_: this->K_list){
				float cos_theta = K_.transpose() * this->R * this->c;
				float visibility = 1.0f / (1.0f + std::exp(-ks * (cos_theta - std::cos(visibility_limit))));
				max_visibility_in_fov +=  visibility;
			} 
			return max_visibility_in_fov;
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
		// #include <cmath>
		// #include <array>
		// inline float sign(float x) {
		// 	return (x >= 0.0f) ? 1.0f : -1.0f;
		// }

		// std::array<float, 4> rotationMatrixToQuaternion(const float R[9]) {
		// 	const float& r11 = R[0], r12 = R[1], r13 = R[2];
		// 	const float& r21 = R[3], r22 = R[4], r23 = R[5];
		// 	const float& r31 = R[6], r32 = R[7], r33 = R[8];

		// 	// Calculate quaternion components (w, x, y, z)
		// 	float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
		// 	float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
		// 	float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
		// 	float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;

		// 	// Clamp negative values to zero
		// 	q0 = std::max(q0, 0.0f);
		// 	q1 = std::max(q1, 0.0f);
		// 	q2 = std::max(q2, 0.0f);
		// 	q3 = std::max(q3, 0.0f);

		// 	// Take square roots
		// 	q0 = std::sqrt(q0);
		// 	q1 = std::sqrt(q1);
		// 	q2 = std::sqrt(q2);
		// 	q3 = std::sqrt(q3);

		// 	// Determine largest component and set signs
		// 	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		// 		q0 *= 1.0f;
		// 		q1 *= sign(r32 - r23);
		// 		q2 *= sign(r13 - r31);
		// 		q3 *= sign(r21 - r12);
		// 	} else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		// 		q0 *= sign(r32 - r23);
		// 		q1 *= 1.0f;
		// 		q2 *= sign(r21 + r12);
		// 		q3 *= sign(r13 + r31);
		// 	} else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		// 		q0 *= sign(r13 - r31);
		// 		q1 *= sign(r21 + r12);
		// 		q2 *= 1.0f;
		// 		q3 *= sign(r32 + r23);
		// 	} else {
		// 		q0 *= sign(r21 - r12);
		// 		q1 *= sign(r31 + r13);
		// 		q2 *= sign(r32 + r23);
		// 		q3 *= 1.0f;
		// 	}

		// 	// Normalize the quaternion
		// 	const float norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
		// 	return {
		// 		q0/norm,  // w
		// 		q1/norm,  // x
		// 		q2/norm,  // y
		// 		q3/norm   // z
		// 	};
		// }


		void brute_force_search_with_feature_count(void){
	   		// int optimized_max_feature_in_FOV_=0;
	   		// Eigen::Vector3f optimized_FOV_vector=this->R*this->c;
	   		// // print_string("optimized_FOV_vector");
	   		// // print_eigen_v(optimized_FOV_vector);
			// for(Eigen::Vector3f K_: this->K_list){
			// 	if( abs(acos((K_.transpose())*optimized_FOV_vector))<this->visibility_alpha){
			// 		optimized_max_feature_in_FOV_+=1;
			// 	}
			// }

			// this->optimized_max_feature_in_FOV=optimized_max_feature_in_FOV_;
			int count=0;
			for (Eigen::Vector3f direction:this->loader->get_pointcloud()){
				count+=1;
				direction=direction/direction.norm();
				Eigen::Vector3f Ideal;
				Ideal<<0.753056,0.657866,0.0108853;
				Ideal=Ideal/Ideal.norm();

				int max_feature_in_FOV=0;
				
				for(Eigen::Vector3f K_: this->K_list){
					if( abs(acos((K_.transpose())*direction))<this->visibility_alpha){
						max_feature_in_FOV+=1;
					}
				}
				//std::cout<<"max_feature_in_FOV "<<max_feature_in_FOV<<std::endl;
				if (max_feature_in_FOV>this->brute_force_max_feature_in_FOV){
					this->brute_force_max_feature_in_FOV=max_feature_in_FOV;
					this->brute_force_best_vector_w_feature=direction;
				}
			}		
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
			for (Eigen::Vector3f direction:this->loader->get_pointcloud()){
				direction=direction/direction.norm();
				float max_visibility_in_fov = 0.0f;
				for(Eigen::Vector3f K_: this->K_list){
					float cos_theta = K_.transpose() * direction;
					float visibility = 1.0f / (1.0f + std::exp(-ks * (cos_theta - std::cos(this->visibility_alpha))));
					max_visibility_in_fov +=  visibility;
				} 
				//std::cout<<"max_feature_in_FOV "<<max_feature_in_FOV<<std::endl;
				if (max_visibility_in_fov>this->brute_force_max_visibility){
					this->brute_force_max_visibility = max_visibility_in_fov;
					this->brute_force_best_vector_w_vis=direction;
				}
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
			/*ADDED:*/
			int redo =0;
			std::vector<float> visibility_history;
			float max_visibility = 10.0f;
			const int history_window = 3;
			int perturb_after= 5; 
			float J_thresh= 1;
			float vis_thresh= 1;

			float degree_between;
			int feature_count;
			Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
			float momentum = 0.0;
			float step, v, w, coeff;
			float initial_step = 0.001;
			float decay_rate = 0.01;  // Adjust this decay rate for tuning
			float total_feature_count = 0;
			float mean_feature_count= 0;
			float max_feature_count=0;
			int zero_j_count = 0;
			// std::cout<<"starting c"<<this->c<<std::endl;
			for (int i = 0; i < this->max_iteration; i++) {
				Eigen::Vector3f aJ_l;
				aJ_l << 0, 0, 0;
				float residual = 0.0;
				float current_visibility=0.0;
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
						float visibility_alpha;
						float iteration_count = (float)i;
						
						if (iteration_count / this->max_iteration < 0.2) {
							visibility_alpha = this->visibility_alpha_180;
						} else if (iteration_count / this->max_iteration < 0.3) {
							visibility_alpha = this->visibility_alpha_90;
						} else if (iteration_count / this->max_iteration < 0.4) {
							visibility_alpha = this->visibility_alpha_45;
						} else if (iteration_count / this->max_iteration < 0.5) {
							visibility_alpha = this->visibility_alpha_22_5;
						} else {
							visibility_alpha = this->visibility_alpha;
						}
		
						w = (-1.0) * this->ks * (u - cos(this->visibility_alpha)); 
						v = exp(w);
						coeff = (-1.0) * (pow((1 + v), (-2)) * v * (0.0 - this->ks)); 
						F_Jacobian = coeff * J; 
					}
					Eigen::Vector3f aJ = F_Jacobian;
					aJ_l = aJ_l + aJ;
					/*ADDED:error ( visibility)*/
					current_visibility += 1.0/(1.0+v);
				} 
				//++++++++++++++++++++++++++++++++++++++++++++++++++  Done iterating on landmarks ++++++++++++++++++++++++++++++++++++++++++++++++

				/*ADDED:error (visibility)*/
				visibility_history.push_back(current_visibility);
				if (current_visibility > max_visibility) {
					max_visibility = current_visibility;
				}
				if (visibility_history.size() > history_window) {
					visibility_history.erase(visibility_history.begin()); 
				}

				// std::cout<<aJ_l<<std::endl;
				/*______________________________________________________________________________________-logging_________________*/
				this->ajl << aJ_l << std::endl;
				feature_count = this->get_count_in_fov();
				// total_feature_count += feature_count; 
				// mean_feature_count= total_feature_count / i;
				// if (feature_count>max_feature_count){
				// 	max_feature_count= feature_count;
				// }
				degree_between = acos(brute_force.transpose() * rotated_vec) * 180.0 / M_PI;
				
				this->quiversforonepoint << this->cnt << "," << std::to_string(this->ref_point[0]) << "," 
										 << std::to_string(this->ref_point[1]) << "," << std::to_string(this->ref_point[2]) << ","
										 << std::to_string(this->rotated_vec[0]) << "," << std::to_string(this->rotated_vec[1]) << "," 
										 << std::to_string(this->rotated_vec[2]) << "," << feature_count << "," << degree_between << ","
										 << aJ_l.norm() << ","<< step <<"," << current_visibility<< std::endl;


					/*______________________________________________________________________________________-perturb if needed __________________*/

				/*NOTE: desice to ocontinue optimization at t with starting_c1 or break and start with starting_c2 */
				//ADDED: count iterations with low j
				if (aJ_l.norm() < J_thresh) {
					// std::cout<<"ajl norm"<< aJ_l.norm()<<std::endl;
					zero_j_count++;  
				} else {     
					zero_j_count = 0;  
				}
				if (zero_j_count > perturb_after) {
					zero_j_count = 0;  
					float avg_recent_visibility = std::accumulate(visibility_history.begin(), visibility_history.end(), 0.0f) / visibility_history.size();
					std::cout<<"avg recent visibility "<< avg_recent_visibility<<"  , max visibility  "<<max_visibility<<std::endl;
					if (avg_recent_visibility < 0.5 * max_visibility || avg_recent_visibility<50) {
					// if (current_visibility < 0.5 * max_visibility || current_visibility<1) {
						std::cout<<"not going well. go change start c and start over."<<std::endl;
						redo=1;
						break;

					} else{
						redo=0;
						std::cout<<"jacobian zero for some time but vis not less than thresh so optimal -> break"<<std::endl;
						break;
					}
				}

/*___________________________________________ ________________________________________-update R__________________*/
				// step = initial_step / (1 + std::cbrt(aJ_l.norm()));
				step = initial_step; 
				// step = initial_step / (1+ decay_rate * (i+1));
				// step = (initial_step/ (1 + 0.05 * aJ_l.norm())) * exp(-decay_rate * i);
				// step=(1/(this->K_list.size()*M_PI*8));
				aJ_l = step * aJ_l;
				this->R = this->exp_map(aJ_l) * this->R; 
				this->rotated_vec = this->R * this->c;
				// Eigen::Quaternionf quat = matrixToQuaternion(this->R);
/*_____________________________________________________________________________________-logging_________________*/

				// this->quaternions<<quat.w()<< "," << quat.x() << ","<< quat.y() << "," << quat.z() << std::endl;
				this->R_updates_for_ti << this->R * this->c << std::endl;
				if (this->print_path == true) {
					this->pathfile << std::to_string(rotated_vec[0]) << "," << std::to_string(rotated_vec[1]) << "," 
								   << std::to_string(rotated_vec[2]) << std::endl;
				}
			}
		
			this->R_updates_for_ti << "\n\n";
			this->ajl << "\n\n";
			return redo;
		}
		
		/*ADDED: for steepest desent visual*/
		void calculate_error_all_direction_for_each_t() {
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
		float visibility_angle=45;
		double visibility_alpha=visibility_angle*M_PI/180.0;
		double visibility_alpha_180=179.0*M_PI/180.0;
		double visibility_alpha_90=90.0*M_PI/180.0;
		double visibility_alpha_45=90.0*M_PI/180.0;
		double visibility_alpha_22_5=90.0*M_PI/180.0;
		// double visibility_alpha_45=45.0*M_PI/180.0;
		// double visibility_alpha_22_5=22.5*M_PI/180.0;
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
		CloudLoader *loader;
		CloudLoader *loaderrot;
		int brute_force_max_feature_in_FOV;
		float brute_force_max_visibility;
		int optimized_max_feature_in_FOV;
		Eigen::Vector3f brute_force_best_vector_w_vis;
		Eigen::Vector3f brute_force_best_vector_w_feature;

};

#endif // _MANIFOLD_

