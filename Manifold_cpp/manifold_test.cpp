#include <manifold.h>
#include <cloud_loader.h>
#include <monte_carlo.h>
#include <experiment_manager.h>
int main(int argc, char *argv[]){
	std::string a1(argv[1]);
	std::string a2(argv[2]);
	std::string a3(argv[3]);
	//std::stoi(s);
	int resolution= std::stoi(a1);
	bool import_map= (bool) std::stoi(a2);
	int cluster= std::stoi(a3);
	print_float((float)resolution);
	std::cout<<resolution<<std::endl;
	const auto p1 = std::chrono::system_clock::now();
	int time_since_epoc=std::chrono::duration_cast<std::chrono::seconds>(
                   p1.time_since_epoch()).count();
	bool generate_map_only=false; 
	bool generate_direction_and_uncertainty=false;

	// /* NOTE: for monte carlo run */
	// std::string map_name("../../Map/0_map.csv");
	// ExperimentManager( -250, 250, -250, 250, 0,10, "map.csv", cluster, resolution,  1, 10,10, 1,import_map,map_name,generate_map_only,generate_direction_and_uncertainty);
	/* NOTE: for monte_carlo_FIF run */
	std::string map_name("../../Map/two_walls_points_w.csv");
	ExperimentManager( -2.5, 2.5, -2.5, 2.5, -1,1, "map.csv", cluster, resolution,  1, 10,10, 1,import_map,map_name,generate_map_only,generate_direction_and_uncertainty);
																					
}






