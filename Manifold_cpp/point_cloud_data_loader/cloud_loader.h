#ifndef _CLOUDLOADER_
  
// Defines _ANIMALS_ if above
// conditions fails
#define _CLOUDLOADER_

//#include <manifold.h>
#include <random>

class PoseSE3{
public:
	PoseSE3(Eigen::Vector3f p,Eigen::Matrix3f rot){
		this->position=p;
		this->rotation=rot;
	}
	PoseSE3(){	
		this->position<<0,0,0;//zero
		this->rotation<<1,0,0,0,1,0,0,0,1; //identity
	}


	Eigen::Matrix3f get_rotation(){
		return this->rotation;

	}
	void set_rotation(Eigen::Matrix3f rot){
		this->rotation=rot;

	}

	Eigen::Vector3f get_position(){
		return this->position;

	}

	void set_position(Eigen::Vector3f p){
		this->position=p;
	}

private:
	Eigen::Vector3f position;
	Eigen::Matrix3f rotation;

};


class CloudLoader{
public:
	CloudLoader() {
		std::srand(std::time(0)); 
	}

	std::vector<std::string> split_string(std::string s,std::string delimiter){
		size_t last = 0;
		size_t next = 0;
		std::vector<std::string> splitted_string;
		//std::string delimiter=delimiter;
		while ((next = s.find(delimiter, last)) != std::string::npos) {   
			// std::cout << s.substr(last, next-last) << std::endl;
			splitted_string.push_back(s.substr(last, next-last));
			last = next + 1; 
			} 
		splitted_string.push_back(s.substr(last));
		// std::cout << s.substr(last) << std::endl;
		return splitted_string;
	}


	void ImportFromDirUncertaintyFile(std::string data_filename,int downsample_factor,std::string delimiter){
		//std::cout<<"<1> ImportFromDirUncertaintyFile"<<std::endl;
		this->filename=data_filename;

		this->data_file.open (this->filename);

		//std::cout<<"<1.1>"<<std::endl;
		std::string line;
		int count=0;
		while (getline (this->data_file, line)) {
			//std::cout << line<<std::endl;
			//std::cout<<"<1.1> count ="<<count<<std::endl;
			if (count==0){
				count+=1;
				continue;
			}
			if (count%downsample_factor!=0) {
				count+=1;
				continue;
			}
			// 	count+=1
			// 	continue 
		  // Output the text from the file

			//line.pop_back(); //remove the last character
		 	//std::cout << line<<std::endl;
		 	//std::cout<<"<1.2>"<<std::endl;
		 	std::vector<std::string> splitted_line=this->split_string(line,delimiter);
		 	float x=std::stof(splitted_line[0]);
		 	float y=std::stof(splitted_line[1]);
		 	float z=std::stof(splitted_line[2]);
		 	//std::cout<<"<1.3>"<<std::endl;
		 	Eigen::Vector3f v;
			v<<x,y,z;
			//std::cout<<"<1.31>"<<std::endl;
			float uncertainty=std::stof(splitted_line[3]);
			//std::cout<<"<1.32>"<<std::endl;
			this->dir_vector.push_back(v);
			//std::cout<<"<1.33>"<<std::endl;
			this->uncertainty_vector.push_back(uncertainty);
			//std::cout<<"<1.34>"<<std::endl;
			count+=1;
			//std::cout<<"<1.4>"<<std::endl;
		}
		//std::cout<<"<2> ImportFromDirUncertaintyFile"<<std::endl;
		//std::cout<<"<2>"<<std::endl;
		// for (Eigen::Vector3f v:this->set_A){
		// 	print_eigen_v(v);
		// }
		this->data_file.close();
	}


	std::vector<PoseSE3> ImportFromTrajectoryFile(std::string data_filename,int downsample_factor,std::string delimiter){
		std::cout<<"<1> ImportFromTrajectoryFile"<<std::endl;
		std::cout<<"<1> ImportFromTrajectoryFile data_filename "<<data_filename<<std::endl;
		std::vector<PoseSE3> trajectory;

		this->filename=data_filename;
		this->data_file.open (this->filename);

	    if (!this->data_file.is_open()) {
	        std::cerr << "Failed to open the file for reading." << std::endl;
	    } else {
	        // std::cout << "File opened successfully for reading." << std::endl;
	        // You can read from the file here.
	    }
		//std::cout<<"<1.1>"<<std::endl;
		std::string line;
		int count=0;
		while (getline (this->data_file, line)) {
			//std::cout << line<<std::endl;
			//std::cout<<"<1.1> count ="<<count<<std::endl;
			// if (count==0){
			// 	count+=1;
			// 	continue;
			// }
			if (count%downsample_factor!=0) {
				count+=1;
				continue;
			}
			// 	count+=1
			// 	continue 
		  // Output the text from the file

			//line.pop_back(); //remove the last character
		 	//std::cout << line<<std::endl;
		 	//std::cout<<"<1.2>"<<std::endl;
		 	std::vector<std::string> splitted_line=this->split_string(line,delimiter);
		 	float q_w=std::stof(splitted_line[0]);
		 	float q_x=std::stof(splitted_line[1]);
		 	float q_y=std::stof(splitted_line[2]);
		 	float q_z=std::stof(splitted_line[3]);
		 	float x=std::stof(splitted_line[4]);
		 	float y=std::stof(splitted_line[5]);
		 	float z=std::stof(splitted_line[6]);
		 	//std::cout<<"<1.3>"<<std::endl;
		 	Eigen::Vector3f v;
			v<<x,y,z;


		    Eigen::Quaternionf quaternion(q_w, q_x, q_y, q_z);  // Example quaternion (w, x, y, z)
		    // Convert quaternion to rotation matrix
		    Eigen::Matrix3f rotation_matrix = quaternion.toRotationMatrix();

		    PoseSE3 startingpose(v,rotation_matrix);
		    trajectory.push_back(startingpose);

			//std::cout<<"<1.34>"<<std::endl;
			count+=1;
			//std::cout<<"<1.4>"<<std::endl;
		}
		//std::cout<<"<2> ImportFromDirUncertaintyFile"<<std::endl;
		//std::cout<<"<2>"<<std::endl;
		// for (Eigen::Vector3f v:this->set_A){
		// 	print_eigen_v(v);
		// }
		this->data_file.close();
		return trajectory;
	}

	void ImportFromTxtFile(std::string data_filename, std::string delimiter) {
		this->filename = data_filename;
		this->data_file.open(this->filename);
	
		if (!this->data_file.is_open()) {
			std::cerr << "Failed to open the file for reading." << std::endl;
			return;
		}
	
		std::string line;
		while (getline(this->data_file, line)) {
			std::vector<std::string> splitted_line = this->split_string(line, delimiter);
	
			if (splitted_line.size() < 3) {  // Ensure at least 3 values (x, y, z)
				std::cerr << "Skipping invalid line: " << line << std::endl;
				continue;
			}
	
			try {
				float x = std::stof(splitted_line[0])*150.0;
				float y = std::stof(splitted_line[1])*150.0;
				float z = std::stof(splitted_line[2])*10.0;
	
				Eigen::Vector3f v;
				v << x, y, z;
				this->dir_vector.push_back(v);
			} catch (const std::invalid_argument &e) {
				std::cerr << "Skipping invalid line (non-numeric data): " << line << std::endl;
			} catch (const std::out_of_range &e) {
				std::cerr << "Skipping invalid line (out of range): " << line << std::endl;
			}
		}
	
		this->data_file.close();
	}
	

	void ImportFromXyzFile(std::string data_filename,int downsample_factor,bool simple,bool trim_floor,std::string delimiter){
		//std::cout<<"<1> ImportFromXyzFile"<<std::endl;
		this->filename=data_filename;

		this->data_file.open (this->filename);
	    if (!this->data_file.is_open()) {
	        std::cerr << "Failed to open the file for reading." << std::endl;
	    } else {
	        // std::cout << "File opened successfully for reading." << std::endl;
	        // You can read from the file here.
	    }
		//std::cout<<"<1.1>"<<std::endl;
		std::string line;
		int count=0;
		while (getline (this->data_file, line)) {
			/*NOTE:NOTE: i commented */
			if (count==0){
				count+=1;
				// continue;
			}
			if (count%downsample_factor!=0) {
				count+=1;
				continue;
			}
			// 	count+=1
			// 	continue 
		  // Output the text from the file
			// line.pop_back();
		 	//std::cout << line;
		 	std::vector<std::string> splitted_line=this->split_string(line,delimiter);
		 	float x=std::stof(splitted_line[0]);
		 	float y=std::stof(splitted_line[1]);
		 	float z=std::stof(splitted_line[2]);
		 	float r,g,b;
		 	if (simple==false){
			 	r=std::stof(splitted_line[3]);
			 	g=std::stof(splitted_line[4]);
			 	b=std::stof(splitted_line[5]);
		 	}
		 	if (trim_floor==true){
			 	if (z<6){
			 		continue;
			 	}
			}
			Eigen::Vector3f v;
			if (simple==true){
				v<<x,y,z;
			}
			this->set_A.push_back(v);
			count+=1;


		}
		//std::cout<<"<2>"<<std::endl;
		// for (Eigen::Vector3f v:this->set_A){
		// 	print_eigen_v(v);
		// }
		this->data_file.close();
	}

	std::vector<Eigen::Vector3f> get_pointcloud (void){
		return this->set_A;
	}

	float get_pointcloud_size (void){
		std::cout<<"size of pointcloud(twc number)"<<this->set_A.size()<<std::endl;
		return this->set_A.size();
	}


	std::vector<Eigen::Vector3f> get_pointcloud_dir (void){
		return this->dir_vector;
	}

	std::vector<float> get_pointcloud_uncertainty (void){
		return this->uncertainty_vector;
	}
	/*ADDED: for random starting c*/
	Eigen::Vector3f get_random(void){
    	// int random_index = std::rand() % this->set_A.size();
		// return this->set_A[random_index];

		static std::random_device rd;
		static std::mt19937 gen(rd());
		std::uniform_int_distribution<int> dis(0, this->set_A.size() - 1);

		int random_index = dis(gen);
		return this->set_A[random_index];
	}

private:
	std::string filename;
	std::vector<Eigen::Vector3f> set_A;
	std::ifstream data_file;
	std::vector<Eigen::Vector3f> dir_vector;
	std::vector<float> uncertainty_vector;

	//std::vector<Eigen::Matrix3f> set_B;	
};


#endif //_CLOUDLOADER_