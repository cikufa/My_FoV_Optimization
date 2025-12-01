// #include <manifold.h>
// #include <cloud_loader.h>
// #include <monte_carlo.h>
#include <experiment_manager.h>
// #include <trajectory_optimizer.h>
#include <trajectory_optimizer_copy.h>

//NOTE: before running this run 

int main(int argc, char *argv[]){
	std::cout<<"<1>"<<std::endl;
	if(argc!=3){
		std::cout<<"need to supply the base directory's name as sole argument. Usage manifold_test_trajectory <base_dir> <reg_dir>"<<std::endl;
		return 0;
	}
	std::string base_dir(argv[1]);
	std::string reg_dir(argv[2]);
	std::cout<<"<2>"<<std::endl;


	//TRO Trajectory
	std::string fif= "/home/shekoufeh/my_FIF-perception-aware-planning/act_map_exp/localization";
	std::string var_dir="/diagonal/diagonal_none";
	std::string res= "/res_mini";
	std::string output_initial_file(fif+res+var_dir+"/initial_trajectory.txt");
	std::string output_initial_file_ue(fif+res+var_dir+"/initial_trajectory_ue.txt");
	std::string output_initial_file_twc(fif+res+var_dir+"/initial_trajectory_twc.txt");
	std::string output_pointcloud_file(fif+res+var_dir+"/trajectory_pointcloud.csv");
	std::string input_file(fif+"/warehouse_base/sparse/0/points3D.txt");
	std::string input_trajectory_file(fif+res+var_dir+"/stamped_Twc_path_yaw.txt");
	std::string output_trajectory_file(fif+res+var_dir+"/optimized_stamped.txt");
	std::string output_trajectory_file_ue(fif+res+var_dir+"/optimized_stamped_Twc_ue.txt");
	std::string output_trajectory_file_twc(fif+res+var_dir+"/optimized_stamped_Twc.txt");
		
	std::string output_pointcloud_dir_file(" ");
	std::string input_dir_file(" ");


	std::cout<<"<3>"<<std::endl;
	bool use_direction=false;
	bool use_uncertainty=false;
	// TrajectoryOptimizerOnManifold traj_op(output_file,input_file,output_pointcloud_file,use_direction,use_uncertainty,input_dir_file,output_pointcloud_dir_file,input_trajectory_file,output_trajectory_file);
	myTrajectoryOptimizerOnManifold traj_op(output_initial_file, output_initial_file_ue, output_initial_file_twc,
		input_file,output_pointcloud_file,use_direction,use_uncertainty,input_dir_file,
		output_pointcloud_dir_file,input_trajectory_file,output_trajectory_file, output_trajectory_file_ue, output_trajectory_file_twc);

	std::cout<<"<4>"<<std::endl;
	traj_op.optimize(true);
		
	std::cout<<"<5>"<<std::endl;

}
