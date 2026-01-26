// #include <manifold.h>
// #include <cloud_loader.h>
// #include <monte_carlo.h>
#include <experiment_manager.h>
// #include <trajectory_optimizer.h>
#include <trajectory_optimizer_copy.h>
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

void ApplyEnvOverrides(myTrajectoryOptimizerOnManifold& optimizer) {
	int max_iter = 0;
	if (ReadEnvInt("FOV_OPT_MAX_ITER", &max_iter)) {
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
	if (ReadEnvFloat("FOV_OPT_TRAJ_JAC_STEP", &traj_jac_step)) {
		optimizer.set_trajectory_jacobian_step(traj_jac_step);
	}
	bool log_jacobian = false;
	if (ReadEnvBool("FOV_OPT_LOG_JACOBIAN", &log_jacobian)) {
		optimizer.set_log_jacobian(log_jacobian);
	}
}
}  // namespace

int main(int argc, char *argv[]){
	std::cout<<"<1>"<<std::endl;
	if (argc < 3 || argc > 5) {
		std::cout<<"Usage: manifold_test_trajectory <input_dir> <output_dir> [along_path (0/1)] [points3d_path]"<<std::endl;
		return 0;
	}
	std::string input_dir(argv[1]);
	std::string output_dir(argv[2]);
	bool along_path = false;
	if (argc >= 4) {
		along_path = (std::stoi(argv[3]) != 0);
	}
	std::cout<<"<2>"<<std::endl;

	std::string output_pointcloud_file(JoinPath(output_dir, "trajectory_pointcloud.csv"));
	std::string input_file("/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map_exp/localization/warehouse_base/sparse/0/points3D.txt");
	if (argc == 5) {
		input_file = argv[4];
	}


	std::string input_trajectory_file;
	std::string output_trajectory_file;
	std::string output_trajectory_file_ue;
	std::string output_trajectory_file_twc;
	std::string output_initial_file;
	std::string output_initial_file_ue;
	std::string output_initial_file_twc;

	if (along_path) {
		const std::string base_twc = JoinPath(input_dir, "stamped_Twc.txt");
		const std::string generated_twc = JoinPath(output_dir, "stamped_Twc_path_yaw.txt");
		std::vector<TwcSample> base_samples = LoadTwcFile(base_twc);
		std::vector<TwcSample> path_samples = BuildPathYaw(base_samples);
		if (!SaveTwcFile(generated_twc, path_samples)) {
			return 1;
		}
		input_trajectory_file = generated_twc;

		output_trajectory_file = JoinPath(output_dir, "optimized_quivers_path_yaw.txt");
		output_trajectory_file_ue =
			JoinPath(output_dir, "stamped_Twc_ue_path_yaw.txt");
		output_trajectory_file_twc =
			JoinPath(output_dir, "optimized_stamped_Twc_path_yaw.txt");

		output_initial_file = JoinPath(output_dir, "initial_quivers_path_yaw.txt");
		output_initial_file_ue =
			JoinPath(output_dir, "initial_trajectory_ue_path_yaw.txt");
		output_initial_file_twc =
			JoinPath(output_dir, "initial_trajectory_twc_path_yaw.txt");
	} else {
		input_trajectory_file = JoinPath(input_dir, "stamped_Twc.txt");

		output_trajectory_file = JoinPath(output_dir, "optimized_quivers.txt");
		output_trajectory_file_ue = JoinPath(output_dir, "stamped_Twc_ue.txt");
		output_trajectory_file_twc = JoinPath(output_dir, "stamped_Twc.txt");

		output_initial_file = JoinPath(output_dir, "initial_quivers.txt");
		output_initial_file_ue = JoinPath(output_dir, "initial_trajectory_ue.txt");
		output_initial_file_twc = JoinPath(output_dir, "initial_trajectory_twc.txt");
	}
		
		
	std::string output_pointcloud_dir_file(" ");
	std::string input_dir_file(" ");


	std::cout<<"<3>"<<std::endl;
	bool use_direction=false;
	bool use_uncertainty=false;
	// TrajectoryOptimizerOnManifold traj_op(output_file,input_file,output_pointcloud_file,use_direction,use_uncertainty,input_dir_file,output_pointcloud_dir_file,input_trajectory_file,output_trajectory_file);
	myTrajectoryOptimizerOnManifold traj_op(output_initial_file, output_initial_file_ue, output_initial_file_twc,
		input_file,output_pointcloud_file,use_direction,use_uncertainty,input_dir_file,
		output_pointcloud_dir_file,input_trajectory_file,output_trajectory_file, output_trajectory_file_ue, output_trajectory_file_twc);
	ApplyEnvOverrides(traj_op);

	std::cout<<"<4>"<<std::endl;
	traj_op.optimize(true);
		
	std::cout<<"<5>"<<std::endl;

}
