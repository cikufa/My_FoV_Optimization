// #include <manifold.h>
// #include <cloud_loader.h>
// #include <monte_carlo.h>
// #include <trajectory_optimizer.h>
#include <trajectory_optimizer_copy.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
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
	bool log_jacobian = false;
	if (ReadEnvBool("FOV_OPT_LOG_JACOBIAN", &log_jacobian)) {
		optimizer.set_log_jacobian(log_jacobian);
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
}  // namespace

int main(int argc, char *argv[]){
	if (argc < 3 || argc > 5) {
		return 0;
	}
	std::string input_dir(argv[1]);
	std::string output_dir(argv[2]);
	bool along_path = false;
	if (argc >= 4) {
		along_path = (std::stoi(argv[3]) != 0);
	}

	std::string output_pointcloud_file(JoinPath(output_dir, "trajectory_pointcloud.csv"));
	std::string input_file = ResolvePointCloudPath(input_dir, argc, argv);


	std::string input_trajectory_file;
	std::string output_trajectory_file;
	std::string output_trajectory_file_ue;
	std::string output_trajectory_file_twc;
	std::string output_initial_file;
	std::string output_initial_file_ue;
	std::string output_initial_file_twc;
	std::string output_visible_features_file;
	std::string generated_input_trajectory_file;

	CleanupLegacyOutputFiles(output_dir);

	if (along_path) {
		const std::string base_twc = JoinPath(input_dir, "stamped_Twc.txt");
		const std::string generated_twc = JoinPath(output_dir, ".input_stamped_Twc.txt");
		std::vector<TwcSample> base_samples = LoadTwcFile(base_twc);
		std::vector<TwcSample> path_samples = BuildPathYaw(base_samples);
		if (!SaveTwcFile(generated_twc, path_samples)) {
			return 1;
		}
		generated_input_trajectory_file = generated_twc;
		input_trajectory_file = generated_twc;

		output_trajectory_file = "";
		output_trajectory_file_ue =
			JoinPath(output_dir, "stamped_Twc_ue.txt");
		output_trajectory_file_twc =
			JoinPath(output_dir, "stamped_Twc.txt");

		output_initial_file = JoinPath(output_dir, "per_iteration_quivers.txt");
		output_initial_file_ue = "";
		output_initial_file_twc = "";
		output_visible_features_file =
			JoinPath(output_dir, "visible_features_per_iteration.txt");
	} else {
		input_trajectory_file = JoinPath(input_dir, "stamped_Twc.txt");

		output_trajectory_file = "";
		output_trajectory_file_ue = JoinPath(output_dir, "stamped_Twc_ue.txt");
		output_trajectory_file_twc = JoinPath(output_dir, "stamped_Twc.txt");

		output_initial_file = JoinPath(output_dir, "per_iteration_quivers.txt");
		output_initial_file_ue = "";
		output_initial_file_twc = "";
		output_visible_features_file =
			JoinPath(output_dir, "visible_features_per_iteration.txt");
	}

	std::string warm_start_flag = ReadEnvString("FOV_OPT_WARM_START");
	if (!warm_start_flag.empty() && std::stoi(warm_start_flag) != 0) {
		std::string warm_start_file = ReadEnvString("FOV_OPT_WARM_START_FILE");
		std::string candidate = warm_start_file.empty() ? output_trajectory_file_twc : warm_start_file;
		if (!candidate.empty() && FileExists(candidate)) {
			input_trajectory_file = candidate;
		} else {
			std::cerr << "Warm start enabled but file not found: " << candidate << std::endl;
		}
	}
		
		
	std::string output_pointcloud_dir_file(" ");
	std::string input_dir_file(" ");


	bool use_direction=false;
	bool use_uncertainty=false;
	// TrajectoryOptimizerOnManifold traj_op(output_file,input_file,output_pointcloud_file,use_direction,use_uncertainty,input_dir_file,output_pointcloud_dir_file,input_trajectory_file,output_trajectory_file);
	myTrajectoryOptimizerOnManifold traj_op(output_initial_file, output_initial_file_ue, output_initial_file_twc,
		input_file,output_pointcloud_file,use_direction,use_uncertainty,input_dir_file,
		output_pointcloud_dir_file,input_trajectory_file,output_trajectory_file, output_trajectory_file_ue, output_trajectory_file_twc);
	ApplyEnvOverrides(traj_op);
	ApplyVisibleFeatureDumpOverride(traj_op, output_visible_features_file);

	std::string esdf_map_path = ReadEnvString("FOV_OPT_ESDF_PATH");
	if (!esdf_map_path.empty()) {
		std::cout << "=== ESDF Integration ===" << std::endl;
		if (traj_op.loadEsdfMap(esdf_map_path)) {
			ApplyEsdfEnvOverrides(traj_op);
			size_t visible_count = traj_op.prefilterVisiblePoints();
			std::cout << "Pre-filtering complete. Unique visible landmarks="
			          << visible_count << std::endl;
		} else {
			std::cerr << "Warning: Failed to load ESDF from " << esdf_map_path << std::endl;
			std::cerr << "Continuing optimization without occlusion filtering." << std::endl;
		}
	}

	traj_op.optimize(true);
	if (!generated_input_trajectory_file.empty() &&
	    generated_input_trajectory_file != output_trajectory_file_twc) {
		std::remove(generated_input_trajectory_file.c_str());
	}
		

}
