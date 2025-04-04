// #include <manifold.h>
// #include <cloud_loader.h>
// #include <monte_carlo.h>
#include <experiment_manager.h>
#include <trajectory_optimizer.h>



int main(int argc, char *argv[]){

	std::cout<<"<1>"<<std::endl;
	if(argc!=3){
		std::cout<<"need to supply the base directory's name as sole argument. Usage manifold_test_trajectory <base_dir> <reg_dir>"<<std::endl;
		return 0;
	}

	// srand (time(NULL));
	// std::vector<Eigen::Vector3f> points_list;
	// Eigen::Vector3f a;
	// a<<0,1,0;
	// print_string("a");
	// print_eigen_v(a);
	// FovOptimizerOnManifold manifold("FOV_30degree.pdf",true,15.0,false,points_list,a);
	// std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	// manifold.optimize();
	// std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	// std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

	// CloudLoader loader;
	// loader.ImportFromXyzFile("../../KME_planes.xyz",30,true,true);
	// std::string a1(argv[1]);
	// std::string a2(argv[2]);
	// //std::stoi(s);
	// int resolution= std::stoi(a1);
	// bool import_map= (bool) std::stoi(a2);


	// print_float((float)resolution);

	// std::cout<<resolution<<std::endl;

	// //MonteCarloRun run(15,15,2,-450,450,-450,450,0,20,"group1");
	// //run.run_monte_carlo_optimizer();
	// const auto p1 = std::chrono::system_clock::now();
	// int time_since_epoc=std::chrono::duration_cast<std::chrono::seconds>(
 //                   p1.time_since_epoch()).count();
	// ExperimentManager( -450, 450, -450, 450, 0, 20, std::to_string(time_since_epoc)+"_10_clustered_25000_point_map.csv", 10, resolution,  4, 15,  15, 2,import_map,map_name);
	//
	std::string base_dir(argv[1]);
	std::string reg_dir(argv[2]);
	std::cout<<"<2>"<<std::endl;

	// //TRO Trajectory tests, enable feature direction
	// //generate pointcloud
	// if(generate){
	// 	const auto p1 = std::chrono::system_clock::now();
	// 	int time_since_epoc=std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count();
	// 	//ExperimentManager(float x_low,float x_high,float y_low,float y_high,float z_low,float z_high,std::string map_output_filename, int cluster_count,int feature_count_resolution, int map_count,int x_resolution, int y_resolution,int z_resolution,bool load_map, std::string map_name){
	// 	float x_low=-450;
	// 	float x_high=450;
	// 	float y_low=-450;
	// 	float y_high=450;
	// 	float z_low=0;
	// 	float z_high=20;
	// 	std::string map_output_filename=std::to_string(time_since_epoc)+"_10_clustered_25000_point_map_with_direction_and_uncertainty.csv";
	//  	int cluster_count=5;
	//  	int feature_count_resolution=25000;
	//  	int map_count=4;
	//  	int x_resolution=15;
	//  	int y_resolution=15;
	//  	int z_resolution=2;
	//  	bool load_map=false;

	//  	std::string map_name="";
	//  	//std::string map_name("../../Data/completed/0_1676304451_10_clustered_25000_point_map.csv");

	// 	bool generate_map_only=true;
	// 	bool generate_direction_and_uncertainty=true;
	// 	//float x_low,float x_high,float y_low,float y_high,float z_low,float z_high,std::string map_output_filename, int cluster_count,int feature_count_resolution, int map_count,int x_resolution, int y_resolution,int z_resolution,bool load_map, std::string map_name, bool generate_map_only, bool generate_direction_and_uncertainty
	// 	ExperimentManager(x_low, x_high, y_low, y_high, z_low, z_high, map_output_filename, cluster_count, feature_count_resolution,  map_count, x_resolution,  y_resolution, z_resolution,load_map,map_name,generate_map_only,generate_direction_and_uncertainty);
	// }

	//TRO Trajectory
		std::string output_file("./"+base_dir+"/trajectory_history.csv");
		std::string output_pointcloud_file("./"+base_dir+"/trajectory_pointcloud.csv");
		std::string output_pointcloud_dir_file("./"+base_dir+"/trajectory_pointcloud_dir.csv");
		std::string input_file("./"+base_dir+"/sparse/0/stripped_points3D.txt");
		std::string input_dir_file("./"+base_dir+"/sparse/0/view_direction");
		std::string input_trajectory_file("./"+reg_dir+"trajectory.txt");
		std::string output_trajectory_file("./"+reg_dir+"output_trajectory.txt");

		std::cout<<"<3>"<<std::endl;
		bool use_direction=false;
		bool use_uncertainty=false;
		TrajectoryOptimizerOnManifold traj_op(output_file,input_file,output_pointcloud_file,use_direction,use_uncertainty,input_dir_file,output_pointcloud_dir_file,input_trajectory_file,output_trajectory_file);
		std::cout<<"<4>"<<std::endl;
		traj_op.optimize(true);
		
		std::cout<<"<5>"<<std::endl;

}