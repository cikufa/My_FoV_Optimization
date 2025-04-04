#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <time.h> 
#include <vector>

int print_eigen(Eigen::MatrixX3d m)
{
    // Eigen Matrices do have rule to print them with std::cout
    std::cout << m << std::endl;
    return 0;
}

int print_eigen_v(Eigen::Vector3d m)
{
    // Eigen Matrices do have rule to print them with std::cout
    std::cout << m << std::endl;
    return 0;
}

int print_string (std::string s)
{
    std::cout << s << std::endl;
    return 0;	
}
// class Manifold_minimize_angle_among_all_target_points(object):

class FovOptimizerOnManifold{
	public:
		// 	def skew(self,x): #skew-symmetric
		// 	    return np.array([[0, -x[2], x[1]],
		// 	                     [x[2], 0, -x[0]],
		// 	                     [-x[1], x[0], 0]])
		Eigen::MatrixX3d skew(Eigen::Vector3d x){
			Eigen::Matrix3d skew;
			skew << 0, -x[2], x[1],
			     x[2], 0, -x[0],
			     -x[1], x[0], 0;
	        //print_eigen(skew)
	        return skew;
	    }
		// 	def exp_map(self,phi): #rodrigue's equation
		// 		skew_phi=self.skew(phi)
		// 		skew_phi_matrix=np.matrix(skew_phi)
		// 		I=np.eye(3)
		// 		phi_vec=np.matrix(phi)
		// 		phi_norm=np.linalg.norm(phi_vec)
		// 		sin_phi_norm_div_phi_norm=math.sin(phi_norm)/phi_norm
		// 		one_minus_cos_phi_norm_div_phi_norm_square=(1-math.cos(phi_norm))/(phi_norm*phi_norm)
		// 		skew_phi_matrix_square=skew_phi_matrix*skew_phi_matrix
		// 		exp_map=I+sin_phi_norm_div_phi_norm*skew_phi_matrix+one_minus_cos_phi_norm_div_phi_norm_square*skew_phi_matrix_square
		// 		return exp_map
		Eigen::MatrixX3d exp_map(Eigen::Vector3d phi){
			// print_string("exp_map<1>");
			Eigen::MatrixX3d skew_phi_matrix=this->skew(phi);
			Eigen::MatrixX3d I=Eigen::Matrix<double, 3, 3>::Identity();
			// print_eigen(I);
			float phi_norm =phi.norm();
			float sin_phi_norm_div_phi_norm=sin(phi_norm)/phi_norm;	
			// print_string("exp_map<1.5>");
			float one_minus_cos_phi_norm_div_phi_norm_square=(1.0-cos(phi_norm))/(phi_norm*phi_norm);
			// print_string("exp_map<1.6>");
			Eigen::MatrixX3d skew_phi_matrix_square=skew_phi_matrix*skew_phi_matrix;
			
			// print_string("exp_map<1.7>");
			// print_eigen	(sin_phi_norm_div_phi_norm*skew_phi_matrix);
			// print_string("exp_map<1.8>");
			// print_eigen	(one_minus_cos_phi_norm_div_phi_norm_square*skew_phi_matrix_square);
			// print_string("exp_map<1.9>");
			// print_eigen	(sin_phi_norm_div_phi_norm*skew_phi_matrix+one_minus_cos_phi_norm_div_phi_norm_square*skew_phi_matrix_square);
			// print_string("exp_map<2.0>");
			// print_eigen	(I+sin_phi_norm_div_phi_norm*skew_phi_matrix+one_minus_cos_phi_norm_div_phi_norm_square*skew_phi_matrix_square);
			// print_string("exp_map<2.1>");			
			Eigen::MatrixX3d exp_map=I+sin_phi_norm_div_phi_norm*skew_phi_matrix+one_minus_cos_phi_norm_div_phi_norm_square*skew_phi_matrix_square;
			// print_string("exp_map<2>");
			return exp_map;
	    }	
	    // 	def __init__(self,filename,visibility,visibility_angle,importing,points_list,plotting,starting_c=[1,0,0]):
	    FovOptimizerOnManifold(std::string filename_, double visibility_ ,double visibility_angle_,bool importing_, std::vector<Eigen::Vector3d> points_list_, Eigen::Vector3d starting_c_)
	    {
// 		##           data generation
// 		r = R.from_euler('z', 0, degrees=True)

	    
	    this->R= Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitX())
			  * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitY())
			  * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitZ());
  		this->c=starting_c_/starting_c_.norm();
// 		#[p-t]^T R exp(w) c      p-t is 3x1 vector ;R is rotation matrix , c is 3x1 vector
// 		#world frame point
		//self.points_list=[]
 		if (importing_==true){
// 			self.points_list=points_list
 			this->points_list=points_list_;
 		}
// 		else:
// 			for i in range (0,100):
// 				self.points_list.append(np.matrix([random.randint(20,30),random.randint(50,70),random.randint(0,20)]))
// 			for i in range (0,100):
// 				self.points_list.append(np.matrix([random.randint(-30,-20),random.randint(10,20),random.randint(50,100)]))
	    else{
	    	for (int i=0;i<100;i++){
	    		Eigen::Vector3d a;
	    		Eigen::Vector3d b;
	    		a<<rand() % 10+1+20,rand() % 20+1+50,rand() % 20+1+0;
	    		b<<rand() % 10+1-30,rand() % 10+1+10,rand() %50+1+50;
	    		this->points_list.push_back(a); //thisrand() % 10+1
	    		this->points_list.push_back(b);
	    	} 
	    }
// 		self.t=np.matrix([0.0,0.0,0.0])
	    this->t<<0,0,0;
// 		self.K_list=[]
// 		for point in self.points_list:
// 			K__=point-self.t
// 			K__=K__/np.linalg.norm(K__)
// 			self.K_list.append(K__.copy())
	    for(Eigen::Vector3d point: this->points_list){
	    	Eigen::Vector3d K__=point-this->t;
	    	K__=K__/K__.norm();	
	    	this->K_list.push_back(K__);
	    }
// 		self.plotR1=[]
// 		self.plotR2=[]
// 		self.plotR3=[]
// 		self.fig1, self.ax = plt.subplots()
// 		self.ax = plt.axes(projection='3d')
// 		rotated_vec=self.R*self.c
	    Eigen::Vector3d rotated_vec=this->R*this->c;
//> 		self.ax.scatter3D([rotated_vec[0,0]],[rotated_vec[1,0]],[rotated_vec[2,0]],'blue')
//> 		#self.ax.scatter3D([self.R[0,2]],[self.R[1,2]],[self.R[2,2]],'blue')

// 		self.plot_sum_x=[]
// 		self.plot_sum_y=[]
// 		self.plotstep=[]
//> 		self.optimize()
//	    this->optimize()
// 		#self.optimize_visibility_function()
// 		if plotting==True:
// 			self.plotting()
// 		plt.close()
// 		#self.fig1.close()
// 		#self.fig2.close()
	    this->filename=filename_;
	    this->visibility_angle=visibility_angle_;
	    this->optimize_visibility_sigmoid=visibility_;
	    }	

	private:
		int max_iteration=300;
		Eigen::MatrixX3d R=Eigen::Matrix<double, 3, 3>::Identity();
		Eigen::Vector3d c;
		std::vector<Eigen::Vector3d> points_list;
		Eigen::Vector3d t;
		std::vector<Eigen::Vector3d> K_list;
		std::vector<double> plotR1;
		std::vector<double> plotR2;
		std::vector<double> plotR3;
		std::vector<double> plot_sum_x;
		std::vector<double> plot_sum_y;	
		// 		self.fig1=None
		// 		self.fig2=None
		// 		self.ax=None
		// 		self.ax2=None
		// 		self.alpha2=.11
		// 		self.alpha1=.5
		// 		self.alpha0=.39

		bool optimize_visibility=false;
		bool optimize_visibility_sigmoid;
		std::string filename;
		double ks=15;
		float visibility_angle=15.0;
		double visibility_alpha=visibility_angle*M_PI/180.0;
		std::vector<double> plotstep;
		//Eigen::Vector3d starting_c<<1,0,0;

};


int main(){
	//Eigen::Matrix3f m;
	//m = Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitZ())
	//    * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitY());
	   // * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitZ());
	//Eigen::Matrix3f m1=Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitX());
	srand (time(NULL));
	std::vector<Eigen::Vector3d> points_list;
	Eigen::Vector3d a;
	a<<0,0,0;
	FovOptimizerOnManifold manifold("FOV_30degree.pdf",true,15.0,false,points_list,a);
	Eigen::Vector3d phi(1.0,0,0);
	Eigen::MatrixX3d phi_exp=manifold.exp_map(phi);
	print_eigen(phi_exp);


}




// ##           optimization
// 	def optimize(self):
// 		for i in range (0,self.max_iteration):
// 			#print (i)
// 			#plotx.append(i)
// 			aJ_l=[0,0,0]
// 			for K_ in self.K_list:
// 				K=K_*self.R
// 				#c=
// 				#print (K)
// 				#print (K[0,1])
// 				C1=self.c[0,0]
// 				C2=self.c[1,0]
// 				C3=self.c[2,0]
// 				K1=K[0,0]
// 				K2=K[0,1]
// 				K3=K[0,2]
// 				#print (C1,C2,C3)
// 				#print (K1,K2,K3)
// 				#print("list",[C2*K3-C3*K2,C3*K1-C1*K3,C1*K2-C2*K1])
// 				J=np.matrix([C2*K3-C3*K2,C3*K1-C1*K3,C1*K2-C2*K1]).transpose()
// 				F_Jacobian=J
// 				if self.optimize_visibility==True:
// 					KTRC=K_*self.R*self.c
// 					#holder=
// 					F_Jacobian=2*self.alpha2*KTRC[0,0]*J+self.alpha1*J
// 				if self.optimize_visibility_sigmoid==True:
// 					u=KTRC=K_*self.R*self.c
// 					w=(-1.0)*self.ks*(u-math.cos(self.visibility_alpha))
// 					v=math.exp(w)
// 					coeff=(-1.0)*((1+v)**(-2))*v*(0.0-self.ks)
// 					F_Jacobian=coeff*J
// 				#print (J)
// 				alpha=.001
// 				#alpha=(F_Jacobian.transpose()*F_Jacobian)[0,0]
// 				#print("F_Jacobian.transpose()*F_Jacobian",F_Jacobian.transpose()*F_Jacobian)
// 				#print ("alpha",alpha)
// 				if True:
// 					alpha=1
// 				aJ=alpha*F_Jacobian
// 				aJ_l=[aJ_l[0]+aJ[0,0],aJ_l[1]+aJ[1,0],aJ_l[2]+aJ[2,0]]

// 			#print("aJ_l",aJ_l)
// 			#aJ_m=np.matrix(aJ_l)
// 			#JTJ=aJ_m*aJ_m.transpose()
// 			#aJ_m=aJ_m/JTJ
// 			#print("JTJ",JTJ)
// 			#print("aJ_m",aJ_m)
// 			#aj_over_JTJ=[aJ_m[0,0],aJ_m[0,1],aJ_m[0,2]]
// 			#exp_aj=np.identity(3)+skew_aJ#for small aj
// 			if True:
// 				Jaco=np.matrix(aJ_l)
// 				#step=np.linalg.inv((Jaco.transpose())*Jaco+1*np.eye(3))
// 				step=0.0001

// 				#print("step",step)
// 				#if step>.01: ## why need this?
// 				#	step=.01 .ahh ,
// 				#step=.001 # ahh, there is no gauss-newton for general problems outside of least sqaures
// 				Jaco=step*Jaco
// 				#Jaco=Jaco
// 				#print("Jaco",Jaco)
// 				#print("step",step)
// 				#print("aJ_L",aJ_l)

// 				#print()
// 				for jj in range (0,len(aJ_l)):
// 					aJ_l[jj]=Jaco[0,jj]
// 				#print("step aj_L",aJ_l)


// 			self.R=self.exp_map(aJ_l)*self.R
// 			rotated_vec=self.R*self.c
// 			#self.ax.scatter3D([self.rotated_vec[0,0]],[self.rotated_vec[1,0]],[self.rotated_vec[2,0]],'blue')
// 			#self.ax.scatter3D([self.R[0,2]],[self.R[1,2]],[self.R[2,2]],'blue')
// 			self.plotR1.append(rotated_vec[0,0])
// 			self.plotR2.append(rotated_vec[1,0])
// 			self.plotR3.append(rotated_vec[2,0])	
// 			sum_result=0
// 			for K__ in self.K_list:
// 				sum_result+=K__*self.R*self.c
// 			self.plot_sum_x.append(i)
// 			self.plot_sum_y.append(sum_result[0,0])
// 			self.plotstep.append(step)
// 			self.ax.quiver(0, 0, 0, rotated_vec[0,0], rotated_vec[1,0], rotated_vec[2,0], length=1, normalize=True)
// 			if np.linalg.norm(np.matrix(aJ_l))<.01:
// 				break

// 	def plotting(self):
// 		print(self.R)
// 		KK=0
// 		i=0
// 		for K__ in self.K_list:
// 			KK+=K__
// 			self.ax.scatter3D(K__[0,0],K__[0,1],K__[0,2])
// 			i+=1
// 		print(KK/np.linalg.norm(KK))

// 		self.ax.plot3D(self.plotR1,self.plotR2,self.plotR3,'gray')
// 		self.ax.set_xlabel("x")
// 		self.ax.set_ylabel("y")
// 		self.ax.set_zlabel("z")
// 		expected=KK/np.linalg.norm(KK)

// 		self.ax.scatter3D([expected[0,0]],[expected[0,1]],[expected[0,2]],c='r',s=40)

// 		self.fig2,self.ax2 = plt.subplots()
// 		#ax2 = plt.axes(projection='2d')
// 		self.ax2.plot(self.plot_sum_x,self.plot_sum_y)
// 		self.ax2.plot(self.plot_sum_x,self.plotstep)
// 		#for point in points_list:
// 		self.ax.view_init(elev=10., azim=120)
// 		self.ax.view_init(elev=89., azim=150)
// 		self.ax.set_title(self.filename)
// 		self.ax2.set_title("objective function value"+self.filename)
// 		#self.ax2.view_init(elev=10., azim=180.0)
// 		self.fig1.savefig(self.filename)
// 		self.fig2.savefig("objective_"+self.filename)
// 		#plt.show()



