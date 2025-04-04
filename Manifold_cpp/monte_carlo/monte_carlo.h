#ifndef _MONTECARLO_
  
// Defines _ANIMALS_ if above
// conditions fails
#define _MONTECARLO_

#include <manifold.h>
#include <cloud_loader.h>

class MonteCarloRun{
public:
	MonteCarloRun(int x_resolution,int y_resolution,int z_resolution,float x_low,float x_high,float y_low,float y_high,float z_low,float z_high,std::string prefix,std::string mapfilename){
 		this->x_resolution=x_resolution;
 		this->y_resolution=y_resolution;
 		this->z_resolution=z_resolution;
 		this->x_limit[0]=x_low;
 		this->x_limit[1]=x_high;
 		this->y_limit[0]=y_low;
 		this->y_limit[1]=y_high;
 		this->z_limit[0]=z_low;
 		this->z_limit[1]=z_high;

		this->loader=new CloudLoader;
		/*NOTE:I relaxed the map file name for convenient while debuging*/
		this->loader->ImportFromXyzFile(mapfilename,1,true,false,",");
		// std::cout<<"monte carlo, size of data "<<this->loader->get_pointcloud().size()<<std::endl;

		this->optimizer_monte_carlo_total_time_us=0;
		this->optimizer_monte_carlo_average_time_us=0;
		this->brute_force_search_total_time_us=0;
		this->brute_force_search_average_time_us=0;

		this->quiversfile.open("../../Data/"+prefix+"single_run_rotated_quivers.csv");
		this->brute_force_quiversfile.open("../../Data/"+prefix+"single_run_brute_force_rotated_quivers.csv");
		this->montecarlopointsfile.open("../../Data/"+prefix+"montecarlo_points.csv");

		this->optimizer_avg_time_file.open("../../Data/"+prefix+"optimizer_avg_time_file.csv");
		this->brute_force_avg_time_file.open("../../Data/"+prefix+"brute_force_avg_time_file.csv");
		this->optimizer_accuracy_file.open("../../Data/"+prefix+"optimizer_accuracy_file.csv");
		this->mean.open("../../Data/mean.csv" , std::ios::app);
		this->pointslistfile.open("../../Data/pointslistfile.csv" , std::ios::app);
		this->test.open("../../Data/startingc.csv" , std::ios::app);
		

		for (Eigen::Vector3f point: this->loader->get_pointcloud()){
			this->montecarlopointsfile<<point[0]<<","<<point[1]<<","<<point[2]<<","<<std::endl;
		}
		this->populate_indexes();

		/*ADDED: for random staring c*/
		this->loader2=new CloudLoader;
		this->loader2->ImportFromXyzFile("../../brute_force_xyz_indexes_two_degree.csv",1,true,false,",");
	}

	void populate_indexes(void){
		for (int i=0; i<this->x_resolution;i++){
			this->x_index.push_back(this->x_limit[0]+(this->x_limit[1]-this->x_limit[0])*i/(float)this->x_resolution);

		}
		for (int i=0; i<this->y_resolution;i++){
			this->y_index.push_back(this->y_limit[0]+(this->y_limit[1]-this->y_limit[0])*i/(float)this->y_resolution);			
		}
		for (int i=0; i<this->z_resolution;i++){
			this->z_index.push_back(this->z_limit[0]+(this->z_limit[1]-this->z_limit[0])*i/(float)this->z_resolution);			
		}
	}

	void populate_local_indexes(Eigen::Vector3f ref_point){
		Eigen::Vector3f pos=ref_point;
		// print_string("ref_point");
		// print_eigen_v(ref_point);
		this->points_list.clear();
		for (Eigen::Vector3f point: this->loader->get_pointcloud()){
			Eigen::Vector3f point_pos=point-pos;
			point_pos=point_pos/point_pos.norm();
			this->points_list.push_back(point_pos);
		}
	}

	std::pair<Eigen::Vector3f, Eigen::Vector3f> calculate_pointcloud_mean() {
		Eigen::Vector3f total(0, 0, 0);
		for (const Eigen::Vector3f& point : this->loader->get_pointcloud()) {
			total += point;
		}	
		Eigen::Vector3f totalme = total / static_cast<float>(this->loader->get_pointcloud().size());
		Eigen::Vector3f totalchen = total / total.norm();
		return std::make_pair(totalme, totalchen);
	}	
					
	void run_monte_carlo_optimizer(void) {
		int cnt = 0;		
		int time_us;
		for (int i = 0; i < this->x_resolution; i++) {
			for (int j = 0; j < this->y_resolution; j++) {
				for (int k = 0; k < this->z_resolution; k++) {
					Eigen::Vector3f ref_point, starting_c,origin, quiver_head, brute_force_quiver_head_vis,brute_force_quiver_head_feat;
					int brute_force_max_feature_holder, optimized_max_feature_holder;
					float brute_force_max_visibility_holder, optimized_max_visibility_holder;
					float degree_between;
					std::cout << "_______________________________________________________" << std::endl;
					std::cout << "cam pose " << i << j << k << std::endl;
					ref_point << this->x_index[i], this->y_index[j], this->z_index[k];
					this->populate_local_indexes(ref_point);

					Eigen::Vector3f mean_center_of_all_pointsme, mean_center_of_all_pointsch;
					std::tie(mean_center_of_all_pointsme, mean_center_of_all_pointsch) = this->calculate_pointcloud_mean();	
					this->mean<< std::to_string(mean_center_of_all_pointsme[0])<<","<< std::to_string(mean_center_of_all_pointsme[1])<<","<<std::to_string(mean_center_of_all_pointsme[2])<<","<<std::to_string(mean_center_of_all_pointsch[0])<<","<< std::to_string(mean_center_of_all_pointsch[1])<<","<< std::to_string(mean_center_of_all_pointsch[2])<<std::endl;
					// starting_c=mean_center_of_all_pointsme-ref_point;
					/*ADDED: for random staring c*/
					// starting_c= this->loader2->get_random();
					// starting_c[2] = 0.0f;
					// this->manifold=new FovOptimizerOnManifold("FOV_30degree.pdf",true,15.0,true,this->points_list,starting_c,true, ref_point, cnt);

					 /* TODO:---------------------------------------init -----------------------------------------------*/

					/*ADDED: multi start*/
					Eigen::Vector3f base = mean_center_of_all_pointsch - ref_point;
					Eigen::Vector3f flippedXY = base.cwiseProduct(Eigen::Vector3f(-1, -1, 1));  // Flip X and Y

					Eigen::Matrix3f rotX90 = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()).toRotationMatrix();
					Eigen::Matrix3f rotY90 = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY()).toRotationMatrix();
					Eigen::Matrix3f rotZ90 = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()).toRotationMatrix();
					Eigen::Matrix3f rotZ180 = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()).toRotationMatrix();

					std::vector<Eigen::Vector3f> starting_c_candidates = {
						base,             // Original
						rotZ180 * base,        // Flipped version in XY plane
						rotZ90 * base,
						rotZ90.transpose() * base,
						rotX90 * base,    
						rotY90 * base,    
						rotX90.transpose() * base,  
						rotY90.transpose() * base, 
					};
															
					/*ADDED:*/					
					int redo=1;
					int redo_cnt=0;
					while (redo==1)
					{
						starting_c= starting_c_candidates[redo_cnt];
						this->manifold=new FovOptimizerOnManifold("FOV_30degree.pdf",true,15.0,true,this->points_list,starting_c,true, ref_point, cnt);
						if(redo_cnt==0){
							/*--------------------------------------------brute force-------------------------------------------------------*/
							std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
							this->manifold->brute_force_search_with_visibility();
							this->manifold->brute_force_search_with_feature_count();
							std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
							time_us=std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
							this->brute_force_search_total_time_us+=(float)time_us;				
							this->brute_force_avg_time_file<<std::to_string((float)time_us)<<std::endl;
							brute_force_quiver_head_feat=this->manifold->get_brute_force_best_vector_feat();
							brute_force_quiver_head_vis=this->manifold->get_brute_force_best_vector_vis();
							brute_force_max_visibility_holder = this->manifold->get_brute_force_max_visibility();
							brute_force_max_feature_holder = this->manifold->get_brute_force_max_feature();

								/*ADDED: for steepest decent VisulIZATION */
							this->manifold->calculate_error_all_direction_for_each_t(); 
							this->manifold->calculate_J_all_direction_for_each_t(); 
						}
						
						/*--------------------------------------------optimization-------------------------------------------------------*/
						std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
						redo = this->manifold->optimize(brute_force_quiver_head_vis); /*NOTE: bf vis based*/
						std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
						time_us=std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
						this->optimizer_monte_carlo_total_time_us+=(float)time_us;
						quiver_head=(this->manifold->get_R())*starting_c;
						quiver_head=quiver_head/quiver_head.norm();
						optimized_max_feature_holder = this->manifold->get_optimized_max_feature();
						optimized_max_visibility_holder = this->manifold->get_optimized_max_visibility();
						degree_between =acos(brute_force_quiver_head_vis.transpose()*quiver_head)*180.0/M_PI;
						redo_cnt++;
						delete this->manifold;
						
					}
					float visibility_between= brute_force_max_visibility_holder- optimized_max_visibility_holder;
					cnt++;
					this->degree_diff_record.push_back(degree_between);
					this->optimizer_avg_time_file<<std::to_string((float)time_us)<<std::endl;
					this->quiversfile<< std::to_string(ref_point[0])<<","<< std::to_string(ref_point[1])<<","<< std::to_string(ref_point[2])<<","<< std::to_string(quiver_head[0])<<","<< std::to_string(quiver_head[1])<<","<< std::to_string(quiver_head[2])<<std::endl;
					this->optimizer_accuracy_file<<std::to_string(degree_between)<<","<<std::to_string(brute_force_max_visibility_holder)<<","<<std::to_string(optimized_max_visibility_holder)<<","<<visibility_between<<std::endl;
					this->brute_force_quiversfile<< std::to_string(ref_point[0])<<","<< std::to_string(ref_point[1])<<","<< std::to_string(ref_point[2])<<","<< 
					std::to_string(brute_force_quiver_head_feat[0])<<","<< std::to_string(brute_force_quiver_head_feat[1])<<","<< std::to_string(brute_force_quiver_head_feat[2])
					<<","<<std::to_string(brute_force_quiver_head_vis[0])<<","<< std::to_string(brute_force_quiver_head_vis[1])<<","<< std::to_string(brute_force_quiver_head_vis[2])
					<<std::endl;
					
				}
			}
		}
		this->brute_force_search_average_time_us=this->brute_force_search_total_time_us/(this->x_resolution*this->y_resolution*this->z_resolution);
		this->brute_force_avg_time_file<<std::to_string(this->brute_force_search_average_time_us)<<std::endl;
		// std::cout<<"brute_force_monte_carlo_average_time is "<<this->brute_force_search_average_time_us<<" [us]"<<std::endl;

		this->optimizer_monte_carlo_average_time_us=this->optimizer_monte_carlo_total_time_us/(this->x_resolution*this->y_resolution*this->z_resolution);
		// std::cout<<"optimizer_monte_carlo_average_time is "<<this->optimizer_monte_carlo_average_time_us<<" [us]"<<std::endl;
		this->optimizer_avg_time_file<<std::to_string(this->optimizer_monte_carlo_average_time_us)<<std::endl;
	}


private:

 		int x_resolution;
 		int y_resolution;
 		int z_resolution;
 		float x_limit[2];
 		float y_limit[2];
 		float z_limit[2];	
 		std::vector<float> x_index;
 		std::vector<float> y_index;
 		std::vector<float> z_index;

 		FovOptimizerOnManifold *manifold;
 		CloudLoader *loader;
		CloudLoader *loader2;
 		std::vector<Eigen::Vector3f> points_list;
 		float avg_time;
 		float optimizer_monte_carlo_total_time_us; //us
 		float optimizer_monte_carlo_average_time_us; //us
 		float brute_force_search_total_time_us;
  		float brute_force_search_average_time_us;

  		std::vector<float> degree_diff_record;

  		std::string prefix;

 		std::ofstream quiversfile;
		std::ofstream mean;
		std::ofstream pointslistfile;
		std::ofstream test;
 		std::ofstream brute_force_quiversfile;
 		std::ofstream montecarlopointsfile;
		std::ofstream rotation_sample_file;

 		std::ofstream optimizer_avg_time_file;
 		std::ofstream brute_force_avg_time_file;

 		std::ofstream optimizer_accuracy_file;


};


#endif// _MONTECARLO_
