// #include <manifold.h>
// #include <cloud_loader.h>
// #include <monte_carlo.h>
// #include <trajectory_optimizer.h>
#include <trajectory_optimizer_copy.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

//NOTE: before running this run 

namespace {
std::string JoinPath(const std::string& left, const std::string& right) {
	if (left.empty()) {
		return right;
	}
	if (left.back() == '/') {
		return left + right;
	}
	return left + "/" + right;
}

struct TwcSample {
	double t;
	Eigen::Matrix4d Twc;
};

std::vector<TwcSample> LoadTwcFile(const std::string& path) {
	std::vector<TwcSample> samples;
	std::ifstream in(path);
	if (!in.is_open()) {
		std::cerr << "Failed to open Twc file: " << path << std::endl;
		return samples;
	}
	std::string line;
	while (std::getline(in, line)) {
		if (line.empty() || line[0] == '#') {
			continue;
		}
		std::istringstream iss(line);
		std::vector<double> vals;
		double v;
		while (iss >> v) {
			vals.push_back(v);
		}
		if (vals.size() < 17) {
			continue;
		}
		TwcSample sample;
		sample.t = vals[0];
		sample.Twc = Eigen::Matrix4d::Identity();
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				sample.Twc(i, j) = vals[1 + i * 4 + j];
			}
		}
		samples.push_back(sample);
	}
	return samples;
}

std::vector<TwcSample> BuildPathYaw(const std::vector<TwcSample>& in_samples) {
	std::vector<TwcSample> out;
	if (in_samples.empty()) {
		return out;
	}
	Eigen::Vector3d prev_dir(1.0, 0.0, 0.0);
	for (size_t i = 0; i < in_samples.size(); ++i) {
		Eigen::Vector3d pos = in_samples[i].Twc.block<3, 1>(0, 3);
		Eigen::Vector3d dir(0.0, 0.0, 0.0);
		if (i + 1 < in_samples.size()) {
			Eigen::Vector3d next_pos = in_samples[i + 1].Twc.block<3, 1>(0, 3);
			dir = next_pos - pos;
		} else if (i > 0) {
			Eigen::Vector3d prev_pos = in_samples[i - 1].Twc.block<3, 1>(0, 3);
			dir = pos - prev_pos;
		}
		dir.z() = 0.0;
		if (dir.norm() < 1e-6) {
			dir = prev_dir;
		} else {
			dir.normalize();
			prev_dir = dir;
		}
		const Eigen::Vector3d c1(0.0, 0.0, -1.0);
		const Eigen::Vector3d c2(dir.x(), dir.y(), 0.0);
		Eigen::Vector3d c0 = c1.cross(c2);
		if (c0.norm() < 1e-6) {
			c0 = Eigen::Vector3d(1.0, 0.0, 0.0);
		} else {
			c0.normalize();
		}
		Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
		Twc.block<3, 3>(0, 0).col(0) = c0;
		Twc.block<3, 3>(0, 0).col(1) = c1;
		Twc.block<3, 3>(0, 0).col(2) = c2;
		Twc.block<3, 1>(0, 3) = pos;
		out.push_back({in_samples[i].t, Twc});
	}
	return out;
}

bool SaveTwcFile(const std::string& path, const std::vector<TwcSample>& samples) {
	std::ofstream out(path);
	if (!out.is_open()) {
		std::cerr << "Failed to write Twc file: " << path << std::endl;
		return false;
	}
	out << "# transformation matrices - " << samples.size()
	    << " entries: time; mat of size: 4 x 4\n";
	out << std::setprecision(17);
	for (const auto& sample : samples) {
		out << sample.t;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				out << " " << sample.Twc(i, j);
			}
		}
		out << "\n";
	}
	return true;
}

bool ReadEnvFloat(const char* name, float* out) {
	const char* raw = std::getenv(name);
	if (!raw || !*raw) {
		return false;
	}
	char* endptr = nullptr;
	const float value = std::strtof(raw, &endptr);
	if (endptr == raw) {
		return false;
	}
	*out = value;
	return true;
}

bool ReadEnvDouble(const char* name, double* out) {
	const char* raw = std::getenv(name);
	if (!raw || !*raw) {
		return false;
	}
	char* endptr = nullptr;
	const double value = std::strtod(raw, &endptr);
	if (endptr == raw) {
		return false;
	}
	*out = value;
	return true;
}

bool ReadEnvInt(const char* name, int* out) {
	const char* raw = std::getenv(name);
	if (!raw || !*raw) {
		return false;
	}
	char* endptr = nullptr;
	const long value = std::strtol(raw, &endptr, 10);
	if (endptr == raw) {
		return false;
	}
	*out = static_cast<int>(value);
	return true;
}

bool ReadEnvBool(const char* name, bool* out) {
	int value = 0;
	if (!ReadEnvInt(name, &value)) {
		return false;
	}
	*out = (value != 0);
	return true;
}

bool FileExists(const std::string& path) {
	std::ifstream in(path);
	return in.good();
}

void RemoveIfExists(const std::string& path) {
	if (!path.empty()) {
		std::remove(path.c_str());
	}
}

void CleanupLegacyOutputFiles(const std::string& output_dir) {
	const std::vector<std::string> legacy_files = {
		"initial_quivers_path_yaw.txt",
		"initial_quivers_path_yaw_metrics.txt",
		"optimized_quivers_path_yaw.txt",
		"quivers_path_yaw.txt",
		"quivers_path_yaw_metrics.txt",
		"quivers_path_yaw_visible_idx.txt",
		"initial_quivers_path_yaw_visible_idx.txt",
		"visible_features_per_iteration.txt",
		"optimized_stamped_Twc_path_yaw.txt",
		"optimized_stamped_Twc_ue_path_yaw.txt",
		"stamped_Twc_path_yaw.txt",
		"stamped_Twc_ue_path_yaw.txt",
		"initial_trajectory_twc_path_yaw.txt",
		"initial_trajectory_ue_path_yaw.txt",
	};
	for (const std::string& name : legacy_files) {
		RemoveIfExists(JoinPath(output_dir, name));
	}
}

std::string ReadEnvString(const char* name) {
	const char* raw = std::getenv(name);
	if (!raw || !*raw) {
		return std::string();
	}
	return std::string(raw);
}

bool ReadEnvFloatList(const char* name, std::vector<float>* out) {
	const char* raw = std::getenv(name);
	if (!raw || !*raw) {
		return false;
	}
	std::stringstream ss(raw);
	std::string token;
	bool any = false;
	while (std::getline(ss, token, ',')) {
		if (token.empty()) {
			continue;
		}
		char* endptr = nullptr;
		const float value = std::strtof(token.c_str(), &endptr);
		if (endptr == token.c_str()) {
			continue;
		}
		out->push_back(value);
		any = true;
	}
	return any;
}

std::string ResolvePointCloudPath(const std::string& input_dir,
	                               const int argc,
	                               char* argv[]) {
	if (argc == 5) {
		return std::string(argv[4]);
	}
	const std::string env_path = ReadEnvString("FOV_OPT_POINTS_PATH");
	if (!env_path.empty()) {
		return env_path;
	}
	const std::vector<std::string> candidates = {
		JoinPath(input_dir, "sparse/0/stripped_points3D.txt"),
		JoinPath(input_dir, "segmented_pointcloud.txt"),
		JoinPath(input_dir, "sparse/0/points3D.txt"),
	};
	for (const std::string& candidate : candidates) {
		if (FileExists(candidate)) {
			return candidate;
		}
	}
	return candidates.front();
}

std::string ResolvePointCloudPath(const std::string& input_dir,
	                               const std::string& explicit_path) {
	if (!explicit_path.empty()) {
		return explicit_path;
	}
	return ResolvePointCloudPath(input_dir, 0, nullptr);
}

void ApplyEnvOverrides(myTrajectoryOptimizerOnManifold& optimizer) {
	int max_iter = 0;
	if (ReadEnvInt("FOV_OPT_MAX_ITER", &max_iter) ||
	    ReadEnvInt("FOV_OPT_MAX_ITERATION", &max_iter)) {
		optimizer.set_max_iteration(max_iter);
	}
	double ks = 0.0;
	if (ReadEnvDouble("FOV_OPT_KS", &ks)) {
		optimizer.set_ks(ks);
	}
	bool ks_from_visibility = false;
	if (ReadEnvBool("FOV_OPT_KS_FROM_VISIBILITY", &ks_from_visibility)) {
		optimizer.set_ks_from_visibility(ks_from_visibility);
	}
	float ks_transition_deg = 0.0f;
	if (ReadEnvFloat("FOV_OPT_KS_TRANSITION_DEG", &ks_transition_deg)) {
		optimizer.set_ks_transition_deg(ks_transition_deg);
	}
	float vis_angle = 0.0f;
	if (ReadEnvFloat("FOV_OPT_VIS_ANGLE_DEG", &vis_angle)) {
		optimizer.set_visibility_angle_deg(vis_angle);
	}
	float base_step_scale = 0.0f;
	if (ReadEnvFloat("FOV_OPT_BASE_STEP_SCALE", &base_step_scale)) {
		optimizer.set_base_step_scale(base_step_scale);
	}
	float min_step_deg = 0.0f;
	float max_step_deg = 0.0f;
	const bool has_min_step = ReadEnvFloat("FOV_OPT_MIN_STEP_DEG", &min_step_deg);
	const bool has_max_step = ReadEnvFloat("FOV_OPT_MAX_STEP_DEG", &max_step_deg);
	if (has_min_step || has_max_step) {
		optimizer.set_step_limits_deg(min_step_deg, max_step_deg);
	}
	float traj_jac_step = 0.0f;
	if (ReadEnvFloat("FOV_OPT_TRAJ_JAC_STEP", &traj_jac_step) ||
	    ReadEnvFloat("FOV_OPT_TRAJECTORY_JACOBIAN_STEP", &traj_jac_step)) {
		optimizer.set_trajectory_jacobian_step(traj_jac_step);
	}
	std::vector<float> fov_schedule;
	if (ReadEnvFloatList("FOV_OPT_FOV_SCHEDULE", &fov_schedule)) {
		optimizer.set_fov_schedule_deg(fov_schedule);
	}
	const std::string step_norm_mode = ReadEnvString("FOV_OPT_STEP_NORM_MODE");
	if (!step_norm_mode.empty()) {
		optimizer.set_step_norm_mode(step_norm_mode);
	}
	const std::string fov_norm_mode = ReadEnvString("FOV_OPT_FOV_NORM_MODE");
	if (!fov_norm_mode.empty()) {
		optimizer.set_fov_norm_mode(fov_norm_mode);
	}
	const std::string update_frame_mode = ReadEnvString("FOV_OPT_UPDATE_FRAME_MODE");
	if (!update_frame_mode.empty()) {
		optimizer.set_update_frame_mode(update_frame_mode);
	}
	bool adaptive_step_enabled = false;
	if (ReadEnvBool("FOV_OPT_ADAPTIVE_STEP_ENABLED", &adaptive_step_enabled)) {
		optimizer.set_adaptive_step_enabled(adaptive_step_enabled);
	}
	float min_step_decay = 0.0f;
	if (ReadEnvFloat("FOV_OPT_MIN_STEP_DECAY", &min_step_decay)) {
		optimizer.set_min_step_decay(min_step_decay);
	}
	float max_step_decay = 0.0f;
	if (ReadEnvFloat("FOV_OPT_MAX_STEP_DECAY", &max_step_decay)) {
		optimizer.set_max_step_decay(max_step_decay);
	}
	float adapt_max_clip_thresh = 0.0f;
	if (ReadEnvFloat("FOV_OPT_ADAPT_MAX_CLIP_THRESH", &adapt_max_clip_thresh)) {
		optimizer.set_adapt_max_clip_thresh(adapt_max_clip_thresh);
	}
	float adapt_min_clip_thresh = 0.0f;
	if (ReadEnvFloat("FOV_OPT_ADAPT_MIN_CLIP_THRESH", &adapt_min_clip_thresh)) {
		optimizer.set_adapt_min_clip_thresh(adapt_min_clip_thresh);
	}
	float adapt_low_max_clip_thresh = 0.0f;
	if (ReadEnvFloat("FOV_OPT_ADAPT_LOW_MAX_CLIP_THRESH", &adapt_low_max_clip_thresh)) {
		optimizer.set_adapt_low_max_clip_thresh(adapt_low_max_clip_thresh);
	}
	float adapt_low_min_clip_thresh = 0.0f;
	if (ReadEnvFloat("FOV_OPT_ADAPT_LOW_MIN_CLIP_THRESH", &adapt_low_min_clip_thresh)) {
		optimizer.set_adapt_low_min_clip_thresh(adapt_low_min_clip_thresh);
	}
	float adapt_shrink_factor = 0.0f;
	if (ReadEnvFloat("FOV_OPT_ADAPT_SHRINK_FACTOR", &adapt_shrink_factor)) {
		optimizer.set_adapt_shrink_factor(adapt_shrink_factor);
	}
	float adapt_grow_high_min_factor = 0.0f;
	if (ReadEnvFloat("FOV_OPT_ADAPT_GROW_HIGH_MIN_FACTOR", &adapt_grow_high_min_factor)) {
		optimizer.set_adapt_grow_high_min_factor(adapt_grow_high_min_factor);
	}
	float adapt_grow_low_clip_factor = 0.0f;
	if (ReadEnvFloat("FOV_OPT_ADAPT_GROW_LOW_CLIP_FACTOR", &adapt_grow_low_clip_factor)) {
		optimizer.set_adapt_grow_low_clip_factor(adapt_grow_low_clip_factor);
	}
	float adapt_scale_min = 0.0f;
	if (ReadEnvFloat("FOV_OPT_ADAPT_SCALE_MIN", &adapt_scale_min)) {
		optimizer.set_adapt_scale_min(adapt_scale_min);
	}
	float adapt_scale_max = 0.0f;
	if (ReadEnvFloat("FOV_OPT_ADAPT_SCALE_MAX", &adapt_scale_max)) {
		optimizer.set_adapt_scale_max(adapt_scale_max);
	}
	bool log_jacobian = false;
	if (ReadEnvBool("FOV_OPT_LOG_JACOBIAN", &log_jacobian)) {
		optimizer.set_log_jacobian(log_jacobian);
	}
	bool debug_log_enabled = false;
	if (ReadEnvBool("FOV_OPT_DEBUG_LOG_ENABLED", &debug_log_enabled) ||
	    ReadEnvBool("FOV_OPT_DEBUG_LOG", &debug_log_enabled)) {
		optimizer.set_debug_log_enabled(debug_log_enabled);
	}
	const std::string debug_log_path = ReadEnvString("FOV_OPT_DEBUG_LOG_PATH");
	if (!debug_log_path.empty()) {
		optimizer.set_debug_log_path(debug_log_path);
	}
}

void ApplyEsdfEnvOverrides(myTrajectoryOptimizerOnManifold& optimizer) {
	float esdf_threshold = 0.1f;
	ReadEnvFloat("FOV_OPT_ESDF_THRESHOLD", &esdf_threshold);
	bool use_interp = true;
	int use_interp_int = 0;
	if (ReadEnvInt("FOV_OPT_ESDF_USE_INTERP", &use_interp_int)) {
		use_interp = (use_interp_int != 0);
	}
	optimizer.setEsdfConfig(esdf_threshold, use_interp);

	float ray_step_scale = 0.8f;
	ReadEnvFloat("FOV_OPT_ESDF_RAY_STEP_SCALE", &ray_step_scale);
	float ray_min_step = -1.0f;
	ReadEnvFloat("FOV_OPT_ESDF_RAY_MIN_STEP", &ray_min_step);
	float ray_max_step = -1.0f;
	ReadEnvFloat("FOV_OPT_ESDF_RAY_MAX_STEP", &ray_max_step);
	float endpoint_margin = -1.0f;
	ReadEnvFloat("FOV_OPT_ESDF_ENDPOINT_MARGIN", &endpoint_margin);
	bool unknown_is_occluded = true;
	ReadEnvBool("FOV_OPT_ESDF_UNKNOWN_IS_OCCLUDED", &unknown_is_occluded);
	optimizer.setEsdfRaycastConfig(ray_step_scale, ray_min_step, ray_max_step,
		endpoint_margin, unknown_is_occluded);
}

void ApplyOcclusionBackendEnvOverrides(myTrajectoryOptimizerOnManifold& optimizer) {
	const std::string backend = ReadEnvString("FOV_OPT_OCCLUSION_BACKEND");
	if (!backend.empty()) {
		optimizer.set_occlusion_backend(backend);
	}

	float min_dist = 0.0f;
	float max_dist = std::numeric_limits<float>::infinity();
	const bool has_min_dist =
		ReadEnvFloat("FOV_OPT_VISMAP_MIN_DIST", &min_dist) ||
		ReadEnvFloat("FOV_OPT_DEPTHMAP_MIN_DIST", &min_dist);
	const bool has_max_dist =
		ReadEnvFloat("FOV_OPT_VISMAP_MAX_DIST", &max_dist) ||
		ReadEnvFloat("FOV_OPT_DEPTHMAP_MAX_DIST", &max_dist);
	if (has_min_dist || has_max_dist) {
		optimizer.setVisibilityDepthMapRange(min_dist, max_dist);
	}
}

void ApplyVisibleFeatureDumpOverride(myTrajectoryOptimizerOnManifold& optimizer,
	                                 const std::string& default_path) {
	bool dump_visible_features = false;
	if (!ReadEnvBool("FOV_OPT_DUMP_VISIBLE_FEATURES", &dump_visible_features) ||
	    !dump_visible_features) {
		return;
	}
	std::string dump_path = ReadEnvString("FOV_OPT_VISIBLE_FEATURES_PATH");
	if (dump_path.empty()) {
		dump_path = default_path;
	}
	optimizer.set_visible_feature_dump_path(dump_path);
}

struct JobRequest {
	std::string input_dir;
	std::string output_dir;
	bool along_path = false;
	std::string pointcloud_path;
	bool warm_start = false;
	std::string warm_start_file;
};

struct JobTimingSummary {
	double optimization_core_ms = 0.0;
	double occlusion_prefilter_ms = 0.0;
	double esdf_prefilter_ms = 0.0;
	double depthmap_prefilter_ms = 0.0;
	double total_measured_ms = 0.0;
	std::string prefilter_backend = "none";
};

#ifdef FOV_HAVE_ACT_MAP_DEPTHMAP
struct SharedResources {
	act_map::VisibilityCheckerPtr depth_map_visibility_checker;
	std::string depth_map_path;
};

bool InitializeSharedResources(SharedResources* resources) {
	if (!resources) {
		return false;
	}
	std::string depth_map_path = ReadEnvString("FOV_OPT_VISIBILITY_DEPTHMAP_PATH");
	if (depth_map_path.empty()) {
		depth_map_path = ReadEnvString("FOV_OPT_DEPTHMAP_PATH");
	}
	if (depth_map_path.empty()) {
		return true;
	}

	float min_dist = 0.0f;
	float max_dist = std::numeric_limits<float>::infinity();
	ReadEnvFloat("FOV_OPT_VISMAP_MIN_DIST", &min_dist) ||
	    ReadEnvFloat("FOV_OPT_DEPTHMAP_MIN_DIST", &min_dist);
	ReadEnvFloat("FOV_OPT_VISMAP_MAX_DIST", &max_dist) ||
	    ReadEnvFloat("FOV_OPT_DEPTHMAP_MAX_DIST", &max_dist);

	try {
		act_map::DepthMapOptions options;
		act_map::DepthMapPtr depth_map(new act_map::DepthMap(options));
		depth_map->loadDepthLayer(depth_map_path);
		act_map::VisibilityCheckerOptions checker_options;
		checker_options.use_depth_layer_ = true;
		checker_options.depth_layer_proto_fn_ = depth_map_path;
		checker_options.min_dist = min_dist;
		checker_options.max_dist = max_dist;
		resources->depth_map_visibility_checker.reset(
		    new act_map::VisibilityChecker(checker_options, depth_map, nullptr));
		resources->depth_map_path = depth_map_path;
		std::cerr << "Preloaded shared visibility depth map from: "
		          << depth_map_path << std::endl;
		return true;
	} catch (const std::exception& e) {
		std::cerr << "Failed to preload visibility depth map: " << e.what()
		          << std::endl;
		return false;
	}
}
#else
struct SharedResources {
};

bool InitializeSharedResources(SharedResources* /*resources*/) {
	return true;
}
#endif

bool ParseServerJobRequest(const std::string& line, JobRequest* job) {
	if (!job) {
		return false;
	}
	std::vector<std::string> fields;
	std::stringstream ss(line);
	std::string token;
	while (std::getline(ss, token, '\t')) {
		fields.push_back(token);
	}
	if (fields.size() < 5 || fields[0] != "JOB") {
		return false;
	}
	job->input_dir = fields[1];
	job->output_dir = fields[2];
	job->along_path = (fields[3] == "1" || fields[3] == "true");
	job->pointcloud_path = fields[4];
	if (fields.size() >= 6) {
		job->warm_start = (fields[5] == "1" || fields[5] == "true");
	}
	if (fields.size() >= 7) {
		job->warm_start_file = fields[6];
	}
	return true;
}

int RunOptimizationJob(const JobRequest& job, const SharedResources& shared_resources,
	                   JobTimingSummary* timing_summary) {
	if (timing_summary) {
		*timing_summary = JobTimingSummary();
	}

	std::string output_pointcloud_file(JoinPath(job.output_dir, "trajectory_pointcloud.csv"));
	std::string input_file = ResolvePointCloudPath(job.input_dir, job.pointcloud_path);

	std::string input_trajectory_file;
	std::string output_trajectory_file;
	std::string output_trajectory_file_ue;
	std::string output_trajectory_file_twc;
	std::string output_initial_file;
	std::string output_initial_file_ue;
	std::string output_initial_file_twc;
	std::string output_visible_features_file;
	std::string generated_input_trajectory_file;

	CleanupLegacyOutputFiles(job.output_dir);

	if (job.along_path) {
		const std::string base_twc = JoinPath(job.input_dir, "stamped_Twc.txt");
		const std::string generated_twc = JoinPath(job.output_dir, ".input_stamped_Twc.txt");
		std::vector<TwcSample> base_samples = LoadTwcFile(base_twc);
		std::vector<TwcSample> path_samples = BuildPathYaw(base_samples);
		if (!SaveTwcFile(generated_twc, path_samples)) {
			return 1;
		}
		generated_input_trajectory_file = generated_twc;
		input_trajectory_file = generated_twc;

		output_trajectory_file = "";
		output_trajectory_file_ue = JoinPath(job.output_dir, "stamped_Twc_ue.txt");
		output_trajectory_file_twc = JoinPath(job.output_dir, "stamped_Twc.txt");

		output_initial_file = JoinPath(job.output_dir, "per_iteration_quivers.txt");
		output_initial_file_ue = "";
		output_initial_file_twc = "";
		output_visible_features_file =
		    JoinPath(job.output_dir, "visible_features_per_iteration.txt");
	} else {
		input_trajectory_file = JoinPath(job.input_dir, "stamped_Twc.txt");

		output_trajectory_file = "";
		output_trajectory_file_ue = JoinPath(job.output_dir, "stamped_Twc_ue.txt");
		output_trajectory_file_twc = JoinPath(job.output_dir, "stamped_Twc.txt");

		output_initial_file = JoinPath(job.output_dir, "per_iteration_quivers.txt");
		output_initial_file_ue = "";
		output_initial_file_twc = "";
		output_visible_features_file =
		    JoinPath(job.output_dir, "visible_features_per_iteration.txt");
	}

	const bool use_warm_start =
	    job.warm_start ||
	    (!ReadEnvString("FOV_OPT_WARM_START").empty() &&
	     std::stoi(ReadEnvString("FOV_OPT_WARM_START")) != 0);
	if (use_warm_start) {
		std::string warm_start_file = job.warm_start_file;
		if (warm_start_file.empty()) {
			warm_start_file = ReadEnvString("FOV_OPT_WARM_START_FILE");
		}
		std::string candidate =
		    warm_start_file.empty() ? output_trajectory_file_twc : warm_start_file;
		if (!candidate.empty() && FileExists(candidate)) {
			input_trajectory_file = candidate;
		} else {
			std::cerr << "Warm start enabled but file not found: " << candidate
			          << std::endl;
		}
	}

	std::string output_pointcloud_dir_file(" ");
	std::string input_dir_file(" ");

	bool use_direction = false;
	bool use_uncertainty = false;
	myTrajectoryOptimizerOnManifold traj_op(
	    output_initial_file, output_initial_file_ue, output_initial_file_twc,
	    input_file, output_pointcloud_file, use_direction, use_uncertainty,
	    input_dir_file, output_pointcloud_dir_file, input_trajectory_file,
	    output_trajectory_file, output_trajectory_file_ue, output_trajectory_file_twc);
	ApplyEnvOverrides(traj_op);
	ApplyVisibleFeatureDumpOverride(traj_op, output_visible_features_file);
	ApplyOcclusionBackendEnvOverrides(traj_op);

#ifdef FOV_HAVE_ACT_MAP_DEPTHMAP
	if (shared_resources.depth_map_visibility_checker) {
		std::cout << "=== Prebuilt Visibility Map Integration (shared) ==="
		          << std::endl;
		traj_op.setSharedVisibilityDepthMapChecker(
		    shared_resources.depth_map_visibility_checker,
		    shared_resources.depth_map_path);
	}
#endif

	double occlusion_prefilter_ms = 0.0;
	double esdf_prefilter_ms = 0.0;
	double depthmap_prefilter_ms = 0.0;
	std::string prefilter_backend = "none";
	std::string depth_map_path = ReadEnvString("FOV_OPT_VISIBILITY_DEPTHMAP_PATH");
	if (depth_map_path.empty()) {
		depth_map_path = ReadEnvString("FOV_OPT_DEPTHMAP_PATH");
	}
#ifdef FOV_HAVE_ACT_MAP_DEPTHMAP
	if (!shared_resources.depth_map_visibility_checker && !depth_map_path.empty()) {
#else
	if (!depth_map_path.empty()) {
#endif
		std::cout << "=== Prebuilt Visibility Map Integration ===" << std::endl;
		if (!traj_op.loadVisibilityDepthMap(depth_map_path)) {
			std::cerr << "Warning: Failed to load visibility depth map from "
			          << depth_map_path << std::endl;
		}
	}

	std::string esdf_map_path = ReadEnvString("FOV_OPT_ESDF_PATH");
	if (!esdf_map_path.empty()) {
		std::cout << "=== ESDF Integration ===" << std::endl;
		if (traj_op.loadEsdfMap(esdf_map_path)) {
			ApplyEsdfEnvOverrides(traj_op);
		} else {
			std::cerr << "Warning: Failed to load ESDF from " << esdf_map_path
			          << std::endl;
			std::cerr << "Continuing optimization without occlusion filtering."
			          << std::endl;
		}
	}

	if (traj_op.isOcclusionPrefilterEnabled()) {
		const auto prefilter_start = std::chrono::steady_clock::now();
		const size_t visible_count = traj_op.prefilterVisiblePoints();
		const auto prefilter_end = std::chrono::steady_clock::now();
		occlusion_prefilter_ms = std::chrono::duration<double, std::milli>(
		    prefilter_end - prefilter_start).count();
		prefilter_backend = traj_op.getEffectiveOcclusionBackendName();
		if (prefilter_backend == "esdf") {
			esdf_prefilter_ms = occlusion_prefilter_ms;
		} else if (prefilter_backend == "depthmap") {
			depthmap_prefilter_ms = occlusion_prefilter_ms;
		}
		std::cout << "Pre-filtering complete. backend=" << prefilter_backend
		          << ", unique visible landmarks=" << visible_count
		          << ", time_ms=" << occlusion_prefilter_ms << std::endl;
	}

	traj_op.exportInitialTrajectoryReference();
	const auto optimize_start = std::chrono::steady_clock::now();
	traj_op.optimize(false);
	const auto optimize_end = std::chrono::steady_clock::now();
	const double optimization_core_ms = std::chrono::duration<double, std::milli>(
	    optimize_end - optimize_start).count();
	const double total_measured_ms = occlusion_prefilter_ms + optimization_core_ms;
	traj_op.exportOptimizedTrajectoryOutputs();

	std::cout << "Optimization timing: core_ms=" << optimization_core_ms
	          << ", esdf_prefilter_ms=" << esdf_prefilter_ms
	          << ", depthmap_prefilter_ms=" << depthmap_prefilter_ms
	          << ", occlusion_prefilter_ms=" << occlusion_prefilter_ms
	          << ", prefilter_backend=" << prefilter_backend
	          << ", total_ms=" << total_measured_ms << std::endl;

	const std::string timing_path = JoinPath(job.output_dir, "optimization_timing.csv");
	std::ofstream timing_file(timing_path);
	if (timing_file.is_open()) {
		timing_file << "optimization_core_ms,esdf_prefilter_ms,total_measured_ms,"
		               "depthmap_prefilter_ms,occlusion_prefilter_ms,prefilter_backend\n";
		timing_file << std::fixed << std::setprecision(6)
		            << optimization_core_ms << ","
		            << esdf_prefilter_ms << ","
		            << total_measured_ms << ","
		            << depthmap_prefilter_ms << ","
		            << occlusion_prefilter_ms << ","
		            << prefilter_backend << "\n";
	}

	if (!generated_input_trajectory_file.empty() &&
	    generated_input_trajectory_file != output_trajectory_file_twc) {
		std::remove(generated_input_trajectory_file.c_str());
	}

	if (timing_summary) {
		timing_summary->optimization_core_ms = optimization_core_ms;
		timing_summary->occlusion_prefilter_ms = occlusion_prefilter_ms;
		timing_summary->esdf_prefilter_ms = esdf_prefilter_ms;
		timing_summary->depthmap_prefilter_ms = depthmap_prefilter_ms;
		timing_summary->total_measured_ms = total_measured_ms;
		timing_summary->prefilter_backend = prefilter_backend;
	}
	return 0;
}

int RunServerMode() {
	std::ostream protocol(std::cout.rdbuf());
	std::cout.rdbuf(std::cerr.rdbuf());

	SharedResources shared_resources;
	if (!InitializeSharedResources(&shared_resources)) {
		protocol << "INIT_FAILED" << std::endl;
		return 1;
	}

	protocol << "READY" << std::endl;

	std::string line;
	while (std::getline(std::cin, line)) {
		if (line == "EXIT") {
			protocol << "BYE" << std::endl;
			return 0;
		}

		JobRequest job;
		if (!ParseServerJobRequest(line, &job)) {
			protocol << "RESULT\t1\tparse_error" << std::endl;
			continue;
		}

		JobTimingSummary timing_summary;
		const int result =
		    RunOptimizationJob(job, shared_resources, &timing_summary);
		protocol << "RESULT\t" << result << "\t"
		         << timing_summary.total_measured_ms << std::endl;
	}

	return 0;
}
}  // namespace

int main(int argc, char *argv[]){
	if (argc == 2 && std::string(argv[1]) == "--server") {
		return RunServerMode();
	}

	if (argc < 3 || argc > 5) {
		return 0;
	}
	SharedResources shared_resources;
	const JobRequest job = {
	    std::string(argv[1]),
	    std::string(argv[2]),
	    (argc >= 4) ? (std::stoi(argv[3]) != 0) : false,
	    ResolvePointCloudPath(std::string(argv[1]), argc, argv),
	};
	return RunOptimizationJob(job, shared_resources, nullptr);
}
