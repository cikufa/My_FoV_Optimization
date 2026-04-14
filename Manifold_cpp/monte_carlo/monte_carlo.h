#ifndef _MONTECARLO_
  
// Defines _ANIMALS_ if above
// conditions fails
#define _MONTECARLO_

#include <manifold.h>
#include <cloud_loader.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#ifdef _OPENMP
#include <omp.h>
#endif

// Total FoV cone = 30° (half-angle from optical axis = 15°). Same angle for BF, optimization,
// and starting-candidate scoring (replaces the old 45° heuristic).
static constexpr float kMonteCarloFovFullDeg = 30.0f;
static constexpr float kMonteCarloFovHalfAngleDeg = kMonteCarloFovFullDeg * 0.5f;

class MonteCarloRun{
public:
	MonteCarloRun(int x_resolution,int y_resolution,int z_resolution,float x_low,float x_high,float y_low,float y_high,float z_low,float z_high,std::string prefix,std::string mapfilename,std::string posefilename){
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

		this->posefilename = posefilename;
		this->use_pose_file = !posefilename.empty();
		if (this->use_pose_file){
			this->loader3 = new CloudLoader;
			this->loader3->ImportFromXyzFile(posefilename,1,true,false,",");
			this->pose_count_ = static_cast<size_t>(this->loader3->get_pointcloud_size());
		}else{
			this->loader3 = nullptr;
			this->pose_count_ = static_cast<size_t>(this->x_resolution) *
			                     static_cast<size_t>(this->y_resolution) *
			                     static_cast<size_t>(this->z_resolution);
		}

		this->optimizer_monte_carlo_total_time_us=0;
		this->optimizer_monte_carlo_average_time_us=0;
		this->brute_force_search_total_time_us=0;
		this->brute_force_search_average_time_us=0;

		const std::string env_pref = fov_monte_carlo_data_prefix();
		const std::string out_prefix = env_pref.empty() ? prefix : env_pref;

		const std::string quivers_path = "../../Data/" + out_prefix + "single_run_rotated_quivers.csv";
		const std::string brute_force_quivers_path = "../../Data/" + out_prefix + "single_run_brute_force_rotated_quivers.csv";
		const std::string montecarlopoints_path = "../../Data/" + out_prefix + "montecarlo_points.csv";
		const std::string optimizer_avg_path = "../../Data/" + out_prefix + "optimizer_avg_time_file.csv";
		const std::string brute_force_avg_path = "../../Data/" + out_prefix + "brute_force_avg_time_file.csv";
		const std::string mean_path = "../../Data/" + out_prefix + "mean.csv";
		const std::string pointslist_path = "../../Data/" + out_prefix + "pointslistfile.csv";
		const std::string startingc_path = "../../Data/" + out_prefix + "startingc.csv";

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
		this->populate_indexes();

		this->prefix = out_prefix;
		this->bf_objective_ = "both";
		if (const char* bf_obj = std::getenv("FOV_BF_OBJECTIVE")) {
			std::string obj = bf_obj;
			std::transform(obj.begin(), obj.end(), obj.begin(), ::tolower);
			if (obj == "feat" || obj == "vis" || obj == "both") {
				this->bf_objective_ = obj;
			}
		}
		std::string bf_cache_file_override;
		if (const char* bf_env = std::getenv("FOV_BF_CACHE_FILE")) {
			if (bf_env[0] != '\0') {
				bf_cache_file_override = bf_env;
				const size_t slash = bf_cache_file_override.find_last_of("/\\");
				if (slash != std::string::npos) {
					bf_cache_file_override = bf_cache_file_override.substr(slash + 1);
				}
				if (bf_cache_file_override.find("..") != std::string::npos) {
					bf_cache_file_override.clear();
				}
			}
		}
		const char* cache_dir = std::getenv("FOV_BF_CACHE_DIR");
		if (cache_dir && cache_dir[0] != '\0'){
			std::string dir(cache_dir);
			if (!dir.empty() && dir.back() != '/'){
				dir += "/";
			}
			if (!bf_cache_file_override.empty()) {
				this->bf_cache_path = dir + bf_cache_file_override;
			} else {
				this->bf_cache_path = dir + out_prefix + "bf_cache.csv";
			}
		}else{
			if (!bf_cache_file_override.empty()) {
				this->bf_cache_path = "../../Data/" + bf_cache_file_override;
			} else {
				this->bf_cache_path = "../../Data/" + out_prefix + "bf_cache.csv";
			}
		}
		if (this->use_pose_file){
			this->bf_cache_signature = build_bf_cache_signature(mapfilename, posefilename);
		}else{
			this->bf_cache_signature = build_bf_cache_signature(mapfilename);
		}
		this->bf_cache_expected_poses = this->pose_count_;
		init_bf_cache();
		this->start_strategy_ = "single";
		if (const char* start_strategy = std::getenv("FOV_START_STRATEGY")) {
			std::string mode = ToLower(start_strategy);
			if (mode == "single" || mode == "multistart" || mode == "adaptive") {
				this->start_strategy_ = mode;
			}
		}
		this->max_start_candidates_ =
		    fov_env_int("FOV_START_COUNT", 3, 1, 32);
		this->adaptive_min_starts_ =
		    fov_env_int("FOV_START_ADAPTIVE_MIN_STARTS", 2, 1, 32);
		this->adaptive_patience_ =
		    fov_env_int("FOV_START_ADAPTIVE_PATIENCE", 1, 1, 32);
		this->adaptive_min_relative_gain_ =
		    fov_env_float("FOV_START_MIN_REL_GAIN", 0.002f, 0.0f, 1.0f);
		this->use_bf_seed_ = fov_env_flag("FOV_START_INCLUDE_BF_SEED", true);

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
			point_pos=point_pos/point_pos.norm(); //unit quivers
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
		const bool minimal_log = minimal_log_enabled();
		const bool no_io = no_io_enabled();
		bool use_openmp = fov_env_flag("FOV_USE_OPENMP", false);
#ifdef _OPENMP
		if (use_openmp) {
			const int threads = fov_env_int("FOV_NUM_THREADS", 0, 0, 1024);
			if (threads > 0) {
				omp_set_num_threads(threads);
			}
		}
#else
		use_openmp = false;
#endif
		std::vector<Eigen::Vector3f> pose_list;
		if (this->use_pose_file && this->loader3){
			pose_list = this->loader3->get_pointcloud();
		}else{
			pose_list.reserve(this->pose_count_);
			for (int i = 0; i < this->x_resolution; i++) {
				for (int j = 0; j < this->y_resolution; j++) {
					for (int k = 0; k < this->z_resolution; k++) {
						Eigen::Vector3f ref_point;
						ref_point << this->x_index[i], this->y_index[j], this->z_index[k];
						pose_list.push_back(ref_point);
					}
				}
			}
		}

		const std::vector<Eigen::Vector3f> map_points = this->loader->get_pointcloud();

		struct PoseResult {
			Eigen::Vector3f ref;
			Eigen::Vector3f opt_dir;
			Eigen::Vector3f bf_feat_dir;
			Eigen::Vector3f bf_vis_dir;
			float opt_time_us = 0.0f;
			float bf_time_us = 0.0f;
			float bf_time_feat_us = 0.0f;
			float bf_time_vis_us = 0.0f;
		};

		const size_t pose_count = pose_list.size();
		std::vector<PoseResult> results(pose_count);
		const size_t map_point_count = map_points.size();

		Eigen::Vector3f mean_center_of_all_pointsme(0, 0, 0);
		for (const Eigen::Vector3f& point : map_points) {
			mean_center_of_all_pointsme += point;
		}
		if (map_point_count > 0) {
			mean_center_of_all_pointsme /= static_cast<float>(map_point_count);
		}
		Eigen::Vector3f mean_center_of_all_pointsch = mean_center_of_all_pointsme;
		const float mean_norm = mean_center_of_all_pointsch.norm();
		if (mean_norm > 1e-6f) {
			mean_center_of_all_pointsch /= mean_norm;
		}

		double bf_total_us = 0.0;
		double opt_total_us = 0.0;

#pragma omp parallel for if(use_openmp) schedule(dynamic) reduction(+:bf_total_us,opt_total_us)
		for (int idx = 0; idx < static_cast<int>(pose_count); ++idx) {
			const int cnt = idx;
			const Eigen::Vector3f ref_point = pose_list[static_cast<size_t>(idx)];

			if (minimal_log) {
#ifdef _OPENMP
				if (use_openmp) {
#pragma omp critical(fov_log)
					{
						std::cout << "pose " << cnt << std::endl;
					}
				} else {
					std::cout << "pose " << cnt << std::endl;
				}
#else
				std::cout << "pose " << cnt << std::endl;
#endif
			} else {
				if (use_openmp) {
#ifdef _OPENMP
#pragma omp critical(fov_log)
					{
						std::cout << "_______________________________________________________" << std::endl;
						std::cout << "cam pose idx " << cnt << std::endl;
					}
#else
					std::cout << "_______________________________________________________" << std::endl;
					std::cout << "cam pose idx " << cnt << std::endl;
#endif
				} else {
					std::cout << "_______________________________________________________" << std::endl;
					std::cout << "cam pose idx " << cnt << std::endl;
				}
			}

			std::vector<Eigen::Vector3f> local_points_list;
			local_points_list.reserve(map_point_count);
			for (const Eigen::Vector3f& point : map_points) {
				Eigen::Vector3f point_pos = point - ref_point;
				const float norm = point_pos.norm();
				if (norm > 1e-9f) {
					point_pos = point_pos / norm;
				}
				local_points_list.push_back(point_pos);
			}

			/*ADDED: multi start*/
			Eigen::Vector3f base = mean_center_of_all_pointsch - ref_point;
			Eigen::Vector3f base_dir = base;
			if (base_dir.norm() < 1e-6f) {
				base_dir = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
			} else {
				base_dir = base_dir / base_dir.norm();
			}

			// Build a true spherical candidate set around the base direction with
			// 90-degree angular spacing: north pole, 4 equatorial directions, south pole.
			Eigen::Vector3f ref_axis = Eigen::Vector3f::UnitZ();
			if (std::abs(base_dir.dot(ref_axis)) > 0.95f) {
				ref_axis = Eigen::Vector3f::UnitY();
			}
			Eigen::Vector3f tangent_u = ref_axis.cross(base_dir);
			if (tangent_u.norm() < 1e-6f) {
				tangent_u = Eigen::Vector3f::UnitX();
			} else {
				tangent_u = tangent_u / tangent_u.norm();
			}
			Eigen::Vector3f tangent_v = base_dir.cross(tangent_u);
			if (tangent_v.norm() < 1e-6f) {
				tangent_v = Eigen::Vector3f::UnitY();
			} else {
				tangent_v = tangent_v / tangent_v.norm();
			}

			std::vector<Eigen::Vector3f> starting_c_candidates = {
				base_dir,
				tangent_u,
				tangent_v,
				-tangent_u,
				-tangent_v,
				-base_dir,
			};

			// Reorder candidates by quick FOV score (higher first) to reduce bad starts.
			const float cos_alpha =
			    std::cos(kMonteCarloFovHalfAngleDeg * static_cast<float>(M_PI) / 180.0f);
			auto score_dir = [&](const Eigen::Vector3f& dir) -> float {
				float norm = dir.norm();
				if (norm < 1e-6f) {
					return -1e9f;
				}
				Eigen::Vector3f d = dir / norm;
				int count = 0;
				for (const Eigen::Vector3f& k : local_points_list) {
					if (k.dot(d) >= cos_alpha) {
						count++;
					}
				}
				return static_cast<float>(count);
			};
			std::vector<std::pair<float, size_t>> scored;
			scored.reserve(starting_c_candidates.size());
			for (size_t sidx = 0; sidx < starting_c_candidates.size(); ++sidx) {
				scored.emplace_back(score_dir(starting_c_candidates[sidx]), sidx);
			}
			std::stable_sort(scored.begin(), scored.end(),
			                 [](const auto& a, const auto& b) { return a.first > b.first; });
			std::vector<Eigen::Vector3f> ordered_candidates;
			ordered_candidates.reserve(starting_c_candidates.size());
			for (const auto& entry : scored) {
				ordered_candidates.push_back(starting_c_candidates[entry.second]);
			}
			starting_c_candidates.swap(ordered_candidates);
			NormalizeAndDeduplicate(starting_c_candidates);
			if (starting_c_candidates.empty()) {
				starting_c_candidates.push_back(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
			}

			const bool do_vis = (this->bf_objective_ == "vis" || this->bf_objective_ == "both");
			const bool do_feat = (this->bf_objective_ == "feat" || this->bf_objective_ == "both");
			int max_attempts = 1;
			if (this->start_strategy_ == "multistart" || this->start_strategy_ == "adaptive") {
				max_attempts = std::min(
				    static_cast<int>(starting_c_candidates.size()),
				    std::max(1, this->max_start_candidates_));
			}
			if (max_attempts <= 0) {
				max_attempts = 1;
			}
			Eigen::Vector3f quiver_head;
			Eigen::Vector3f brute_force_quiver_head_vis(0, 0, 0);
			Eigen::Vector3f brute_force_quiver_head_feat(0, 0, 0);
			int opt_time_us_total = 0;
			int bf_time_us = 0;
			int bf_time_feat_us = 0;
			int bf_time_vis_us = 0;
			bool bf_ready = false;
			float best_visibility = -std::numeric_limits<float>::infinity();
			int best_feature_count = -1;
			int stagnant_attempts = 0;

			for (int attempt_idx = 0; attempt_idx < max_attempts; ++attempt_idx) {
				Eigen::Vector3f starting_c = starting_c_candidates[attempt_idx];
				FovOptimizerOnManifold* manifold =
					new FovOptimizerOnManifold("FOV_30degree.pdf", true,
					                           static_cast<double>(kMonteCarloFovHalfAngleDeg), true,
					                           local_points_list, starting_c, true,
					                           ref_point, cnt);
				if (!bf_ready) {
					/*--------------------------------------------brute force-------------------------------------------------------*/
					bool used_cache = false;
					if (bf_cache_loaded_ && static_cast<size_t>(cnt) < bf_cache_entries_.size()) {
						const BruteForceCacheEntry& entry = bf_cache_entries_[cnt];
						brute_force_quiver_head_feat = entry.bf_feat;
						brute_force_quiver_head_vis = entry.bf_vis;
						bf_time_us = static_cast<int>(entry.bf_time_us);
						bf_time_feat_us = static_cast<int>(entry.bf_time_feat_us);
						bf_time_vis_us = static_cast<int>(entry.bf_time_vis_us);
						if (bf_time_feat_us == 0 && bf_time_vis_us == 0) {
							if (do_feat && !do_vis) {
								bf_time_feat_us = bf_time_us;
							} else if (do_vis && !do_feat) {
								bf_time_vis_us = bf_time_us;
							}
						}
						used_cache = true;
						bf_ready = true;
					}
					if (!used_cache) {
						if (minimal_log) {
#ifdef _OPENMP
							if (use_openmp) {
#pragma omp critical(fov_log)
								{
									std::cout << "pose " << cnt << " brute force quiver calculation..." << std::endl;
								}
							} else {
								std::cout << "pose " << cnt << " brute force quiver calculation..." << std::endl;
							}
#else
							std::cout << "pose " << cnt << " brute force quiver calculation..." << std::endl;
#endif
						}
						bf_time_feat_us = 0;
						bf_time_vis_us = 0;
						if (do_vis) {
							std::chrono::steady_clock::time_point begin_vis = std::chrono::steady_clock::now();
							manifold->brute_force_search_with_visibility();
							std::chrono::steady_clock::time_point end_vis = std::chrono::steady_clock::now();
							bf_time_vis_us = static_cast<int>(std::chrono::duration_cast<std::chrono::microseconds>(end_vis - begin_vis).count());
						}
						if (do_feat) {
							std::chrono::steady_clock::time_point begin_feat = std::chrono::steady_clock::now();
							manifold->brute_force_search_with_feature_count();
							std::chrono::steady_clock::time_point end_feat = std::chrono::steady_clock::now();
							bf_time_feat_us = static_cast<int>(std::chrono::duration_cast<std::chrono::microseconds>(end_feat - begin_feat).count());
						}
						bf_time_us = bf_time_feat_us + bf_time_vis_us;
						if (do_feat) {
							brute_force_quiver_head_feat = manifold->get_brute_force_best_vector_feat();
						}
						if (do_vis) {
							brute_force_quiver_head_vis = manifold->get_brute_force_best_vector_vis();
						}
						// In single-objective mode, mirror the chosen BF result to both slots so
						// downstream outputs/cache stay structurally compatible.
						if (!do_feat && do_vis) {
							brute_force_quiver_head_feat = brute_force_quiver_head_vis;
						}
						if (!do_vis && do_feat) {
							brute_force_quiver_head_vis = brute_force_quiver_head_feat;
						}
						bf_ready = true;

						if (bf_cache_out_.is_open()) {
							BruteForceCacheEntry entry;
							entry.ref = ref_point;
							entry.bf_feat = brute_force_quiver_head_feat;
							entry.bf_vis = brute_force_quiver_head_vis;
							entry.bf_time_us = static_cast<float>(bf_time_us);
							entry.bf_time_feat_us = static_cast<float>(bf_time_feat_us);
							entry.bf_time_vis_us = static_cast<float>(bf_time_vis_us);
#ifdef _OPENMP
							if (use_openmp) {
#pragma omp critical(fov_bf_cache)
								{
									append_bf_cache(entry);
								}
							} else {
								append_bf_cache(entry);
							}
#else
							append_bf_cache(entry);
#endif
						}

						if (!no_io) {
							/*ADDED: for steepest decent VisulIZATION */
							manifold->calculate_error_all_direction_for_each_t();
							manifold->calculate_J_all_direction_for_each_t();
						}
					}
				}
				if (attempt_idx == 0 &&
				    this->start_strategy_ != "single" &&
				    this->use_bf_seed_ && do_vis &&
				    brute_force_quiver_head_vis.norm() > 1e-6f) {
					starting_c_candidates.insert(starting_c_candidates.begin(), brute_force_quiver_head_vis);
					NormalizeAndDeduplicate(starting_c_candidates);
					max_attempts = std::min(
					    static_cast<int>(starting_c_candidates.size()),
					    std::max(1, (this->start_strategy_ == "single") ? 1 : this->max_start_candidates_));
					delete manifold;
					attempt_idx = -1;
					continue;
				}

				/*--------------------------------------------optimization-------------------------------------------------------*/
				if (minimal_log) {
#ifdef _OPENMP
					if (use_openmp) {
#pragma omp critical(fov_log)
						{
							std::cout << "pose " << cnt << " optimizing..." << std::endl;
						}
					} else {
						std::cout << "pose " << cnt << " optimizing..." << std::endl;
					}
#else
					std::cout << "pose " << cnt << " optimizing..." << std::endl;
#endif
				}
				std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
				manifold->optimize(brute_force_quiver_head_vis); /*NOTE: bf vis based*/
				std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
				const int opt_time_us = static_cast<int>(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count());
				opt_time_us_total += opt_time_us;
				Eigen::Vector3f candidate_quiver_head = (manifold->get_R()) * starting_c;
				if (candidate_quiver_head.norm() > 1e-6f) {
					candidate_quiver_head = candidate_quiver_head / candidate_quiver_head.norm();
				}
				const float prev_best_visibility = best_visibility;
				const float candidate_visibility = manifold->get_optimized_max_visibility();
				const int candidate_feature_count = manifold->get_count_in_fov();
				const bool is_better =
				    (candidate_visibility > best_visibility + 1e-6f) ||
				    (std::abs(candidate_visibility - best_visibility) <= 1e-6f &&
				     candidate_feature_count > best_feature_count);
				if (is_better) {
					best_visibility = candidate_visibility;
					best_feature_count = candidate_feature_count;
					quiver_head = candidate_quiver_head;
				}
				if (this->test.is_open()) {
					this->test << starting_c[0] << "," << starting_c[1] << "," << starting_c[2] << std::endl;
				}
				if (this->start_strategy_ == "adaptive" &&
				    attempt_idx + 1 >= std::min(max_attempts, this->adaptive_min_starts_)) {
					bool enough_progress = false;
					if (std::isfinite(prev_best_visibility) && is_better) {
						const float denom = std::max(1.0f, std::abs(prev_best_visibility));
						const float rel_gain = (best_visibility - prev_best_visibility) / denom;
						enough_progress = rel_gain >= this->adaptive_min_relative_gain_;
					}
					if (enough_progress) {
						stagnant_attempts = 0;
					} else {
						stagnant_attempts++;
					}
					if (stagnant_attempts >= this->adaptive_patience_) {
						delete manifold;
						break;
					}
				}
				delete manifold;
			}

			results[static_cast<size_t>(idx)] = {
				ref_point,
				quiver_head,
				brute_force_quiver_head_feat,
				brute_force_quiver_head_vis,
				static_cast<float>(opt_time_us_total),
				static_cast<float>(bf_time_us),
				static_cast<float>(bf_time_feat_us),
				static_cast<float>(bf_time_vis_us)
			};

			bf_total_us += static_cast<double>(bf_time_us);
			opt_total_us += static_cast<double>(opt_time_us_total);
		}

		for (size_t idx = 0; idx < pose_count; ++idx) {
			const PoseResult& result = results[idx];
			this->mean << std::to_string(mean_center_of_all_pointsme[0]) << ","
			           << std::to_string(mean_center_of_all_pointsme[1]) << ","
			           << std::to_string(mean_center_of_all_pointsme[2]) << ","
			           << std::to_string(mean_center_of_all_pointsch[0]) << ","
			           << std::to_string(mean_center_of_all_pointsch[1]) << ","
			           << std::to_string(mean_center_of_all_pointsch[2]) << std::endl;
			this->optimizer_avg_time_file << std::to_string(result.opt_time_us) << std::endl;
			this->brute_force_avg_time_file << std::to_string(result.bf_time_us) << std::endl;
			this->quiversfile << std::to_string(result.ref[0]) << ","
			                  << std::to_string(result.ref[1]) << ","
			                  << std::to_string(result.ref[2]) << ","
			                  << std::to_string(result.opt_dir[0]) << ","
			                  << std::to_string(result.opt_dir[1]) << ","
			                  << std::to_string(result.opt_dir[2]) << std::endl;
			this->brute_force_quiversfile << std::to_string(result.ref[0]) << ","
			                              << std::to_string(result.ref[1]) << ","
			                              << std::to_string(result.ref[2]) << ","
			                              << std::to_string(result.bf_feat_dir[0]) << ","
			                              << std::to_string(result.bf_feat_dir[1]) << ","
			                              << std::to_string(result.bf_feat_dir[2]) << ","
			                              << std::to_string(result.bf_vis_dir[0]) << ","
			                              << std::to_string(result.bf_vis_dir[1]) << ","
			                              << std::to_string(result.bf_vis_dir[2]) << std::endl;
		}

		this->brute_force_search_total_time_us = static_cast<float>(bf_total_us);
		this->optimizer_monte_carlo_total_time_us = static_cast<float>(opt_total_us);

		size_t denom = pose_count > 0 ? pose_count : 1;
		this->brute_force_search_average_time_us = this->brute_force_search_total_time_us / denom;
		this->brute_force_avg_time_file << std::to_string(this->brute_force_search_average_time_us) << std::endl;

		this->optimizer_monte_carlo_average_time_us = this->optimizer_monte_carlo_total_time_us / denom;
		this->optimizer_avg_time_file << std::to_string(this->optimizer_monte_carlo_average_time_us) << std::endl;
	}


private:
		static std::string ToLower(const std::string& value){
			std::string out = value;
			std::transform(out.begin(), out.end(), out.begin(), ::tolower);
			return out;
		}

		static void NormalizeAndDeduplicate(std::vector<Eigen::Vector3f>& dirs){
			std::vector<Eigen::Vector3f> unique_dirs;
			unique_dirs.reserve(dirs.size());
			for (Eigen::Vector3f dir : dirs){
				if (dir.norm() < 1e-6f){
					continue;
				}
				dir = dir / dir.norm();
				bool duplicate = false;
				for (const Eigen::Vector3f& existing : unique_dirs){
					if (existing.dot(dir) > 0.9995f){
						duplicate = true;
						break;
					}
				}
				if (!duplicate){
					unique_dirs.push_back(dir);
				}
			}
			dirs.swap(unique_dirs);
		}

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
		bool use_pose_file = false;
		std::string posefilename;
		size_t pose_count_ = 0;

		std::string prefix;
		std::string bf_objective_;
		std::string start_strategy_;
		int max_start_candidates_ = 3;
		int adaptive_min_starts_ = 2;
		int adaptive_patience_ = 1;
		float adaptive_min_relative_gain_ = 0.002f;
		bool use_bf_seed_ = true;

 		std::ofstream quiversfile;
		std::ofstream mean;
		std::ofstream pointslistfile;
		std::ofstream test;
 		std::ofstream brute_force_quiversfile;
 		std::ofstream montecarlopointsfile;
		std::ofstream rotation_sample_file;

		std::ofstream optimizer_avg_time_file;
		std::ofstream brute_force_avg_time_file;

		struct BruteForceCacheEntry {
			Eigen::Vector3f ref;
			Eigen::Vector3f bf_feat;
			Eigen::Vector3f bf_vis;
			float bf_time_us = 0.0f;
			float bf_time_feat_us = 0.0f;
			float bf_time_vis_us = 0.0f;
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

		std::string build_bf_cache_signature(const std::string& mapfilename){
			std::ostringstream ss;
			ss << "map=" << mapfilename
			   << ";grid=" << this->x_resolution << "," << this->y_resolution << "," << this->z_resolution
			   << ";bounds=" << this->x_limit[0] << "," << this->x_limit[1]
			   << "," << this->y_limit[0] << "," << this->y_limit[1]
			   << "," << this->z_limit[0] << "," << this->z_limit[1]
			   << ";bf_objective=" << this->bf_objective_;
			return ss.str();
		}

		std::string build_bf_cache_signature(const std::string& mapfilename, const std::string& posefilename){
			std::ostringstream ss;
			ss << "map=" << mapfilename << ";pose=" << posefilename
			   << ";bf_objective=" << this->bf_objective_;
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
				bf_cache_out_ << "# columns: ref_x,ref_y,ref_z,bf_feat_x,bf_feat_y,bf_feat_z,bf_vis_x,bf_vis_y,bf_vis_z,bf_time_us,bf_time_feat_us,bf_time_vis_us" << std::endl;
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
				std::cerr << "[MonteCarloRun] BF cache signature mismatch for "
				          << this->bf_cache_path << std::endl
				          << "Expected: " << expected << std::endl
				          << "Found:    " << header << std::endl;
				std::exit(1);
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
				entry.bf_time_us = vals.size() >= 10 ? static_cast<float>(vals[9]) : 0.0f;
				entry.bf_time_feat_us = vals.size() >= 11 ? static_cast<float>(vals[10]) : 0.0f;
				entry.bf_time_vis_us = vals.size() >= 12 ? static_cast<float>(vals[11]) : 0.0f;
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
			             << "," << entry.bf_time_us
			             << "," << entry.bf_time_feat_us
			             << "," << entry.bf_time_vis_us
			             << std::endl;
		}

};


#endif// _MONTECARLO_
