#ifndef _MONTECARLOFIF_
  
// Defines _ANIMALS_ if above
// conditions fails
#define _MONTECARLFIF_

#include <manifold.h>
#include <cloud_loader.h>
#include <fstream>
#include <sstream>
#include <string>

class MonteCarloRun_FIF{
public:
	MonteCarloRun_FIF(int x_resolution,int y_resolution,int z_resolution,float x_low,float x_high,float y_low,float y_high,float z_low,float z_high,std::string prefix,std::string mapfilename, std::string posefilename){

		this->loader=new CloudLoader;
		/*NOTE:I relaxed the map file name for convenient while debuging*/
		this->loader->ImportFromXyzFile(mapfilename,1,true,false,",");

		this->optimizer_monte_carlo_total_time_us=0;
		this->optimizer_monte_carlo_average_time_us=0;
		this->brute_force_search_total_time_us=0;
		this->brute_force_search_average_time_us=0;

		const std::string quivers_path = "../../Data/" + prefix + "single_run_rotated_quivers.csv";
		const std::string brute_force_quivers_path = "../../Data/" + prefix + "single_run_brute_force_rotated_quivers.csv";
		const std::string montecarlopoints_path = "../../Data/" + prefix + "montecarlo_points.csv";
		const std::string optimizer_avg_path = "../../Data/" + prefix + "optimizer_avg_time_file.csv";
		const std::string brute_force_avg_path = "../../Data/" + prefix + "brute_force_avg_time_file.csv";
		const std::string mean_path = "../../Data/mean.csv";
		const std::string pointslist_path = "../../Data/pointslistfile.csv";
		const std::string startingc_path = "../../Data/startingc.csv";

		this->quiversfile.open(quivers_path);
		this->brute_force_quiversfile.open(brute_force_quivers_path);
		this->montecarlopointsfile.open(montecarlopoints_path);

		this->optimizer_avg_time_file.open(optimizer_avg_path);
		this->brute_force_avg_time_file.open(brute_force_avg_path);
		this->mean.open(mean_path, std::ios::app);
		this->pointslistfile.open(pointslist_path, std::ios::app);
		this->test.open(startingc_path, std::ios::app);

		write_header_if_empty(quivers_path, this->quiversfile,
		                      "ref_x,ref_y,ref_z,opt_dir_x,opt_dir_y,opt_dir_z");
		write_header_if_empty(brute_force_quivers_path, this->brute_force_quiversfile,
		                      "ref_x,ref_y,ref_z,bf_feat_x,bf_feat_y,bf_feat_z,bf_vis_x,bf_vis_y,bf_vis_z");
		write_header_if_empty(montecarlopoints_path, this->montecarlopointsfile, "x,y,z");
		write_header_if_empty(optimizer_avg_path, this->optimizer_avg_time_file, "time_us");
		write_header_if_empty(brute_force_avg_path, this->brute_force_avg_time_file, "time_us");
		write_header_if_empty(mean_path, this->mean,
		                      "mean_me_x,mean_me_y,mean_me_z,mean_ch_x,mean_ch_y,mean_ch_z");
		write_header_if_empty(pointslist_path, this->pointslistfile, "x,y,z");
		write_header_if_empty(startingc_path, this->test, "x,y,z");

		for (Eigen::Vector3f point: this->loader->get_pointcloud()){
			this->montecarlopointsfile<<point[0]<<","<<point[1]<<","<<point[2]<<","<<std::endl;
		}
		// this->populate_indexes();  /*NOTE: commented for FIF map run*/

		/*ADDED: for random staring c*/
		this->loader2=new CloudLoader;
		this->loader2->ImportFromXyzFile("../../brute_force_xyz_indexes_two_degree.csv",1,true,false,",");
		this->loader3=new CloudLoader;
		this->loader3->ImportFromXyzFile(posefilename,1,true,false,",");

		this->prefix = prefix;
		this->bf_cache_path = "../../Data/" + prefix + "bf_cache.csv";
		this->bf_cache_signature = build_bf_cache_signature(mapfilename, posefilename);
		this->bf_cache_expected_poses = static_cast<size_t>(this->loader3->get_pointcloud_size());
		init_bf_cache();

		
	}

	void populate_local_indexes(Eigen::Vector3f ref_point){
		Eigen::Vector3f pos=ref_point;
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
		int time_us = 0;
		Eigen::Vector3f ref_point, starting_c,origin, quiver_head, brute_force_quiver_head_vis,brute_force_quiver_head_feat;
		for (Eigen::Vector3f ref_point:this->loader3->get_pointcloud()){
			std::cout << "_______________________________________________________" << std::endl;
			std::cout << "cam pose:  "<<ref_point<< std::endl;

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
					bool used_cache = false;
					if (bf_cache_loaded_ && static_cast<size_t>(cnt) < bf_cache_entries_.size()){
						const BruteForceCacheEntry& entry = bf_cache_entries_[cnt];
						brute_force_quiver_head_feat = entry.bf_feat;
						brute_force_quiver_head_vis = entry.bf_vis;
						time_us = 0;
						used_cache = true;
					}
					if (!used_cache){
						std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
						this->manifold->brute_force_search_with_visibility();
						this->manifold->brute_force_search_with_feature_count();
						std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
						time_us=std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
						brute_force_quiver_head_feat=this->manifold->get_brute_force_best_vector_feat();
						brute_force_quiver_head_vis=this->manifold->get_brute_force_best_vector_vis();

						if (bf_cache_out_.is_open()){
							BruteForceCacheEntry entry;
							entry.ref = ref_point;
							entry.bf_feat = brute_force_quiver_head_feat;
							entry.bf_vis = brute_force_quiver_head_vis;
							append_bf_cache(entry);
						}
					}
					this->brute_force_search_total_time_us+=(float)time_us;				
					this->brute_force_avg_time_file<<std::to_string((float)time_us)<<std::endl;
				}
				
				/*--------------------------------------------optimization-------------------------------------------------------*/
				std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
				redo = this->manifold->optimize(brute_force_quiver_head_vis); /*NOTE: bf vis based*/
				std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
				time_us=std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
				this->optimizer_monte_carlo_total_time_us+=(float)time_us;
				quiver_head=(this->manifold->get_R())*starting_c;
				quiver_head=quiver_head/quiver_head.norm();
				redo_cnt++;
				delete this->manifold;
				
			}
			cnt++;
			this->optimizer_avg_time_file<<std::to_string((float)time_us)<<std::endl;
			this->quiversfile<< std::to_string(ref_point[0])<<","<< std::to_string(ref_point[1])<<","<< std::to_string(ref_point[2])<<","<< std::to_string(quiver_head[0])<<","<< std::to_string(quiver_head[1])<<","<< std::to_string(quiver_head[2])<<std::endl;
			this->brute_force_quiversfile<< std::to_string(ref_point[0])<<","<< std::to_string(ref_point[1])<<","<< std::to_string(ref_point[2])<<","<< 
			std::to_string(brute_force_quiver_head_feat[0])<<","<< std::to_string(brute_force_quiver_head_feat[1])<<","<< std::to_string(brute_force_quiver_head_feat[2])
			<<","<<std::to_string(brute_force_quiver_head_vis[0])<<","<< std::to_string(brute_force_quiver_head_vis[1])<<","<< std::to_string(brute_force_quiver_head_vis[2])
			<<std::endl;
			
			/*ADDED: for steepest decent VisulIZATION */
			this->manifold->calculate_error_all_direction_for_each_t(); 
			this->manifold->calculate_J_all_direction_for_each_t(); 

		}
		this->brute_force_search_average_time_us=this->brute_force_search_total_time_us/this->loader3->get_pointcloud_size();
		this->brute_force_avg_time_file<<std::to_string(this->brute_force_search_average_time_us)<<std::endl;

		this->optimizer_monte_carlo_average_time_us=this->optimizer_monte_carlo_total_time_us/this->loader3->get_pointcloud_size();
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
		CloudLoader *loader3;
 		std::vector<Eigen::Vector3f> points_list;
 		float avg_time;
 		float optimizer_monte_carlo_total_time_us; //us
 		float optimizer_monte_carlo_average_time_us; //us
 		float brute_force_search_total_time_us;
  		float brute_force_search_average_time_us;

  		std::string prefix;

 		std::ofstream quiversfile;
		std::ofstream mean;
		std::ofstream pointslistfile;
		std::ofstream test;
 		std::ofstream brute_force_quiversfile;
 		std::ofstream montecarlopointsfile;
		std::ofstream rotation_sample_file;
		std::ifstream FIFrefpoint;

 		std::ofstream optimizer_avg_time_file;
 		std::ofstream brute_force_avg_time_file;

		struct BruteForceCacheEntry {
			Eigen::Vector3f ref;
			Eigen::Vector3f bf_feat;
			Eigen::Vector3f bf_vis;
		};

		std::vector<BruteForceCacheEntry> bf_cache_entries_;
		std::string bf_cache_path;
		std::string bf_cache_signature;
		size_t bf_cache_expected_poses = 0;
		bool bf_cache_loaded_ = false;
		std::ofstream bf_cache_out_;

		bool file_is_empty(const std::string& path){
			std::ifstream in(path, std::ios::ate | std::ios::binary);
			if (!in.good()){
				return true;
			}
			return in.tellg() == 0;
		}

		void write_header_if_empty(const std::string& path, std::ofstream& stream, const std::string& header){
			if (!stream.is_open()){
				return;
			}
			if (file_is_empty(path)){
				stream << header << std::endl;
			}
		}

		std::string build_bf_cache_signature(const std::string& mapfilename, const std::string& posefilename){
			std::ostringstream ss;
			ss << "map=" << mapfilename
			   << ";poses=" << posefilename
			   << ";count=" << this->loader3->get_pointcloud_size();
			return ss.str();
		}

		void init_bf_cache(){
			if (load_bf_cache()){
				if (bf_cache_entries_.size() >= bf_cache_expected_poses){
					return;
				}
				bf_cache_out_.open(this->bf_cache_path, std::ios::app);
				return;
			}
			bf_cache_entries_.clear();
			bf_cache_out_.open(this->bf_cache_path, std::ios::out);
			if (bf_cache_out_.is_open()){
				bf_cache_out_ << "# signature:" << this->bf_cache_signature << std::endl;
				bf_cache_out_ << "# columns: ref_x,ref_y,ref_z,bf_feat_x,bf_feat_y,bf_feat_z,bf_vis_x,bf_vis_y,bf_vis_z" << std::endl;
			}
		}

		bool load_bf_cache(){
			std::ifstream in(this->bf_cache_path);
			if (!in.good()){
				return false;
			}
			std::string header;
			if (!std::getline(in, header)){
				return false;
			}
			std::string expected = "# signature:" + this->bf_cache_signature;
			if (header != expected){
				return false;
			}
			std::string line;
			while (std::getline(in, line)){
				if (line.empty() || line[0] == '#'){
					continue;
				}
				std::stringstream ss(line);
				std::string token;
				std::vector<double> vals;
				while (std::getline(ss, token, ',')){
					if (token.empty()){
						continue;
					}
					vals.push_back(std::stod(token));
				}
				if (vals.size() < 9){
					continue;
				}
				BruteForceCacheEntry entry;
				entry.ref = Eigen::Vector3f(vals[0], vals[1], vals[2]);
				entry.bf_feat = Eigen::Vector3f(vals[3], vals[4], vals[5]);
				entry.bf_vis = Eigen::Vector3f(vals[6], vals[7], vals[8]);
				bf_cache_entries_.push_back(entry);
			}
			bf_cache_loaded_ = true;
			return true;
		}

		void append_bf_cache(const BruteForceCacheEntry& entry){
			if (!bf_cache_out_.is_open()){
				return;
			}
			bf_cache_out_ << entry.ref[0] << "," << entry.ref[1] << "," << entry.ref[2] << ","
			             << entry.bf_feat[0] << "," << entry.bf_feat[1] << "," << entry.bf_feat[2] << ","
			             << entry.bf_vis[0] << "," << entry.bf_vis[1] << "," << entry.bf_vis[2]
			             << std::endl;
		}

};


#endif// _MONTECARLOFIF_
