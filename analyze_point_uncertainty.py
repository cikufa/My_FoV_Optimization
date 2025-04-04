#This is a copy of import and plot of the trajectory data. but this script also analyzes the point uncertaitnty 

import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import random
from  matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import copy
from matplotlib.lines import Line2D
import matplotlib
import cmath
from scipy.stats import norm 
import time

import sys


class seed(object):
	def __init__(self,depth_mean,depth_min,starting_translation,bearing,original_3D_point,inverse,uncertainty):
		self.starting_translation=starting_translation
		self.bearing=bearing
		self.a=10.0
		self.b=10.0
		self.mu=None
		self.z_range=None
		self.sigma2=None
		self.original_3D_point=None
		if inverse:
			self.mu=1.0/depth_mean
			self.z_range=1.0/depth_min
			self.sigma2=self.z_range*self.z_range/36.0
			self.original_3D_point=original_3D_point
		else:
			self.mu=depth_mean
			self.z_range=depth_min
			self.sigma2=self.z_range*self.z_range*36.0
			self.original_3D_point=original_3D_point

class simulate_point_uncertainty(object):

	def __init__(self,prefix,ISSAC_SIMUALTION,base_dir):
		self.trajectory_pointcloud_set=[]
		self.trajectory_pointcloud_dir_uncertainty_set=[]
		self.trajectory_quiver_set=[]
		self.trajectory_quiver_superset=[]
		self.seed_of_points=[]
		self.trajectory_start=None
		self.scene_min=0.5
		self.scene_max=100
		self.outlier_rate=0.35
		self.HARDCODE_SEED_DIRECTION=True
		self.DEBUG=False
		self.prefix=prefix
		self.FOV=20.0  #half of FOV
		self.average_sigma=100.0
		self.view_index=None
		self.plot_both=False
		self.INVERSE=True
		self.max_import_sigma=0.0

		self.ISSAC_SIMUALTION=ISSAC_SIMUALTION
		self.base_dir=base_dir

	def import_from_xyz_file(self,data_filename,downsample_factor):
		self.file=open(data_filename,"r")
		lines=self.file.readlines()
		count=0
		print(data_filename,len(lines))
		for line in lines:
			if not count%downsample_factor==0:
				count+=1
				continue
			line=line.strip("\n")
			splitted_line=line.split(",")
			#if not splitted_line[-1].isnumeric():
			x=float(splitted_line[0])
			y=float(splitted_line[1])
			z=float(splitted_line[2])
			self.trajectory_pointcloud_set.append([x,y,z])
			#call initialize_point_seed() with hard coded or preset directions
			count+=1

	def import_from_pointcloud_dir_file(self,data_filename,downsample_factor):
		self.file=open(data_filename,"r")
		lines=self.file.readlines()
		count=0
		print(data_filename,len(lines))
		
		for line in lines:
			if not count%downsample_factor==0:
				count+=1
				continue
			line=line.strip("\n")
			splitted_line=line.split(",")
			#if not splitted_line[-1].isnumeric():
			x=float(splitted_line[0])
			y=float(splitted_line[1])
			z=float(splitted_line[2])

				#time.sleep(100)
			uncertainty=float(splitted_line[3])*1.0
			#if data_filename=="trajectory_pointcloud_dir.csv":
			self.trajectory_pointcloud_dir_uncertainty_set.append([x,y,z,uncertainty])
			#call initialize_point_seed() with hard coded or preset directions
			count+=1



	def import_from_trajectory_file(self,data_filename,downsample_factor):
		self.file=open(data_filename,"r")
		lines=self.file.readlines()
		count=0
		print(data_filename,len(lines))
		self.trajectory_quiver_superset=[]


		for line in lines:
			#print(line)
			#print("counter",counter)
			#print("downsample_factor",downsample_factor)
			#print("counter mod downsample_factor==0",counter%downsample_factor==0)

			if len(line)==1:
				self.trajectory_quiver_superset.append(self.trajectory_quiver_set)
				self.trajectory_quiver_set=[]
				count=0
				continue


			if not count%downsample_factor==0:
				count+=1
				continue			

			line=line.strip("\n")
			splitted_line=line.split(",")
			print("splitted_line",splitted_line)
			#if not splitted_line[-1].isnumeric():
			x=float(splitted_line[0])
			y=float(splitted_line[1])
			z=float(splitted_line[2])
			x_head=float(splitted_line[3])
			y_head=float(splitted_line[4])
			z_head=float(splitted_line[5])
			if self.ISSAC_SIMUALTION:
				#print("self.ISSAC_SIMUALTION")
				print("[x,y,z]",[x,y,z])
				v=np.matrix([x_head,y_head,z_head])
				r = R.from_euler('z', 90, degrees=True)
				v_=r.as_matrix()*v.transpose()
				x_head=v_[0,0]
				y_head=v_[1,0]
				z_head=v_[2,0]
				print("[x,y,z]",[x,y,z])

			self.trajectory_quiver_set.append([x,y,z,x_head,y_head,z_head])
			print("appended")
			count+=1
		#print("len(self.trajectory_quiver_superset)",len(self.trajectory_quiver_superset))
		#time.sleep(100)


	def initialize_point_seed(self):#Line 40 call this function
		#if hard code
		# print("len(self.trajectory_quiver_superset)",len(self.trajectory_quiver_superset))
		# print("len(self.trajectory_quiver_superset[0])",len(self.trajectory_quiver_superset[0]))
		# print("len(self.trajectory_quiver_superset[1])",len(self.trajectory_quiver_superset[1]))
		# print("len(self.trajectory_quiver_superset[1])",len(self.trajectory_quiver_superset[-1]))
		# print("len(self.trajectory_quiver_superset[0])",len(self.trajectory_quiver_superset[0][0]))

		self.trajectory_start=[self.trajectory_quiver_superset[0][0][0],self.trajectory_quiver_superset[0][0][1],self.trajectory_quiver_superset[0][0][2]]
		if self.DEBUG:
			print("self.trajectory_start",self.trajectory_start)
		
		if self.HARDCODE_SEED_DIRECTION:
			#initialize hard coded direction according to first trajectory point
			for i in range(0,len(self.trajectory_pointcloud_set)): 
			#for point in self.trajectory_pointcloud_set:
				point=self.trajectory_pointcloud_set[i]
				bearing_uncertainty=self.trajectory_pointcloud_dir_uncertainty_set[i]
				bearing=[bearing_uncertainty[0],bearing_uncertainty[1],bearing_uncertainty[2]]
				depth_mean=100
				depth_min=4
				starting_translation=self.trajectory_start
				original_3D_point=point
				uncertainty=bearing_uncertainty[3]
				s=seed(depth_mean,depth_min,starting_translation,bearing,original_3D_point,self.INVERSE,uncertainty)
				self.seed_of_points.append(s)

	def simulate_point_uncertainty_at_each_point(self,view_index,downsample_factor):
		## for each view after the initial view:
		self.view_index=view_index
		trajectory_quiver_set=self.trajectory_quiver_superset[0]
		if view_index=="initial":
			trajectory_quiver_set=self.trajectory_quiver_superset[0]
		elif view_index=="final":
			trajectory_quiver_set=self.trajectory_quiver_superset[-1]

		index=0
		for trajectory_quiver in trajectory_quiver_set:
			if not index%downsample_factor==0:
				index+=1
				continue
			if index==0:
				index+=1
				continue
			## for each point:
			#call update_point_uncertainty function
			trajectory_waypoint_quiver=trajectory_quiver
			self.update_point_uncertainty(trajectory_waypoint_quiver)
			index+=1
			print("index",index)


	def simulate_z_observation(self,seed):
		#Mixture uniform& Gaussian .25 outleir rate
		unform_or_gaussian_s = np.random.uniform(0,1,1)
		if(unform_or_gaussian_s[0]>=self.outlier_rate):
			z_mean=np.linalg.norm(np.matrix(seed.original_3D_point)-np.matrix(seed.starting_translation))
			mu, sigma = z_mean, 0.1 # mean and standard deviation
			s = np.random.normal(mu, sigma, 1)
			return s[0]
		else:
			s = np.random.uniform(self.scene_min,self.scene_max,1)
			return s[0]
	def visible(self,trajectory_waypont_quiver,three_d_point,fov):
		#get local coordinate
		p_3d=np.matrix(three_d_point)
		t=np.matrix([trajectory_waypont_quiver[0],trajectory_waypont_quiver[1],trajectory_waypont_quiver[2]])
		f=np.matrix([trajectory_waypont_quiver[3],trajectory_waypont_quiver[4],trajectory_waypont_quiver[5]])
		
		p_3d_f=p_3d-t

		#normalize
		p_3d_f=p_3d_f/np.linalg.norm(p_3d_f)
		f=f/np.linalg.norm(f)
		#dot product
		cos_=p_3d_f*f.transpose()
		#print("cos_",cos_)
		#cos_=abs(cos_[0,0])
		cos_=cos_[0,0]
		#compare cos
		fov_cos=np.cos(fov*np.pi/180.0)
		if cos_<=fov_cos:
			return False
		else:
			return True




	def update_point_uncertainty(self,trajectory_waypont_quiver):
		#take seed direction of feature
		#update according to SVO code
		sigma2_sum=0.0
		three_d_point_error_sum=0.0
		print("trajectory_waypont_quiver",trajectory_waypont_quiver)
		for i in range (0,len(self.seed_of_points)):
			if not self.visible(trajectory_waypont_quiver,self.seed_of_points[i].original_3D_point,self.FOV):
				#print("continue, not visible")
				continue
			if self.DEBUG:
				print("i",i)
				print("len(self.seed_of_points)",len(self.seed_of_points))
			# const double focal_length = frame->cam_->errorMultiplier2();
			focal_length = 80.0 #70 pixels
			# double px_noise = 1.0;
			pixel_noise=1.0
			# double px_error_angle = atan(px_noise/(2.0*focal_length))*2.0; // law of chord (sehnensatz)

			px_error_angle = cmath.atan(pixel_noise/(2.0*focal_length))*2.0;
			px_error_angle=px_error_angle.real
			#px_error_angle= math.atan2(pixel_noise,(2.0*focal_length))*2.0;
			#print("px_error_angle_cmath",px_error_angle_cmath)
			if self.DEBUG:
				print("px_error_angle",px_error_angle)
			z=self.simulate_z_observation(self.seed_of_points[i])
			# // compute tau
			# double tau = computeTau(T_ref_cur, it->ftr->f, z, px_error_angle);
			tau=self.computeTau(self.seed_of_points[i].starting_translation, self.seed_of_points[i].bearing, z, px_error_angle);
			# double tau_inverse = 0.5 * (1.0/max(0.0000001, z-tau) - 1.0/(z+tau));
			if self.DEBUG:
				print("after compute tau, tau is ",tau,"z is ", z)
			tau_inverse = 0.5 * (1.0/max(0.0000001, z-tau) - 1.0/(z+tau))
			if self.DEBUG:
				print("after compute tau, tau inverse is ",tau_inverse)
			# // update the estimate
			# updateSeed(1./z, tau_inverse*tau_inverse, &*it);
			if self.DEBUG:
				print("self.seed_of_points[i].sigma2",self.seed_of_points[i].sigma2)
				print("self.seed_of_points[i].mu",self.seed_of_points[i].mu)
				print("self.seed_of_points[i].a",self.seed_of_points[i].a)
				print("self.seed_of_points[i].b",self.seed_of_points[i].b)
			if self.INVERSE:
				self.updateSeed(1.0/z, tau_inverse*tau_inverse, self.seed_of_points[i])
			else:
				self.updateSeed(z, tau*tau, self.seed_of_points[i])
			if self.DEBUG:
				print("after update self.seed_of_points[i].sigma2",self.seed_of_points[i].sigma2)
				print("after update self.seed_of_points[i].mu",self.seed_of_points[i].mu)
				print("after update self.seed_of_points[i].a",self.seed_of_points[i].a)
				print("after update self.seed_of_points[i].b",self.seed_of_points[i].b)
			sigma2_sum+=self.seed_of_points[i].sigma2
			f=np.matrix(self.seed_of_points[i].bearing)
			t=np.matrix(self.seed_of_points[i].starting_translation)
			z=1.0/self.seed_of_points[i].mu
			f=f/np.linalg.norm(f)
			p=t+f*z
			error=np.matrix([p[0,0]-self.seed_of_points[i].original_3D_point[0],p[0,1]-self.seed_of_points[i].original_3D_point[1],p[0,2]-self.seed_of_points[i].original_3D_point[2]])

			three_d_point_error=np.linalg.norm(error)
			three_d_point_error_sum+=three_d_point_error
		print("average_sigma2",sigma2_sum/len(self.seed_of_points))
		self.average_sigma=np.sqrt(sigma2_sum/len(self.seed_of_points))
		print("average_3d_point_error",three_d_point_error_sum/len(self.seed_of_points))
	def computeTau(self,translation,bearing,z_range,px_error_angle):
		#Vector3d t(T_ref_cur.translation());// this is seed's frame translation
		if self.DEBUG:
			print("in compute tau")
			print("translation",translation)
			print("bearing",bearing)
			print("z_range",z_range)
			print("px_error_angle",px_error_angle)



		t=np.matrix(translation)
		#Vector3d a = f*z-t;
		f=np.matrix(bearing)
		z=z_range
		f_norm=np.linalg.norm(f)
		f=f/f_norm
		a=f*z-t
		#double t_norm = t.norm();
		t_norm=np.linalg.norm(t)
		#double a_norm = a.norm();
		a_norm = np.linalg.norm(a)
		if self.DEBUG:
			print("in compute tau, segment 2")
			print("f",f)
			print("t",t)
			print("a",a)
			print("t_norm",t_norm)
			print("a_norm",a_norm)
			print("z",z)



		#double alpha = acos(f.dot(t)/t_norm); // dot product
		alpha=cmath.acos((f*t.transpose())[0,0]/t_norm)## use cmath acos here, use cmath atan at atan2
		#double beta = acos(a.dot(-t)/(t_norm*a_norm)); // dot product
		alpha=alpha.real
		beta=cmath.acos((a*-t.transpose())[0,0]/(t_norm*a_norm))
		beta=beta.real
		if self.DEBUG:
			print("alpha",alpha)
			print("beta",beta)

		#double beta_plus = beta + px_error_angle;
		beta_plus = beta + px_error_angle
		#double gamma_plus = PI-alpha-beta_plus; // triangle angles sum to PI
		gamma_plus = np.pi-alpha-beta_plus
		#double z_plus = t_norm*sin(beta_plus)/sin(gamma_plus); // law of sines
		z_plus = t_norm*np.sin(beta_plus)/np.sin(gamma_plus)
		#return (z_plus - z); // tau


		return z_plus-z;


	def updateSeed_non_inverse(self,x,tau2,seed):
		# void DepthFilter::updateSeed(const float x, const float tau2, Seed* seed)
		# float norm_scale = sqrt(seed->sigma2 + tau2);
		if self.DEBUG:
			print("seed.sigma2",seed.sigma2)
			print("tau2",tau2)
		norm_scale=np.sqrt(seed.sigma2+tau2)
		# if(std::isnan(norm_scale))
		if math.isnan(norm_scale):
			return
		# return;
		# boost::math::normal_distribution<float> nd(seed->mu, norm_scale);
		mu, sigma = seed.mu, norm_scale # mean and standard deviation
		#s = np.random.normal(mu, sigma, 1)
		# float s2 = 1./(1./seed->sigma2 + 1./tau2);
		s2= 1.0/(1.0/seed.sigma2 + 1.0/tau2)
		# float m = s2*(seed->mu/seed->sigma2 + x/tau2);
		m= s2*(seed.mu/seed.sigma2 + x/tau2)
		# float C1 = seed->a/(seed->a+seed->b) * boost::math::pdf(nd, x);
		probability_pdf=0.0
		INVERSE=True;
		if INVERSE:
			mean = seed.mu
			std = norm_scale
			if self.DEBUG:
				print("x",x)
				print("mean",mean)
				print("std",std)
			probability_pdf = norm.pdf(x, loc=mean, scale=std) 
			if self.DEBUG:
				print("INVERSE probability_pdf",probability_pdf)
				probability_pdf_ref = norm.pdf(mean, loc=mean, scale=std)
				print("INVERSE reference probability_pdf",probability_pdf_ref )
		else:
			mean = 1.0/seed.mu
			std = 1.0/norm_scale
			if self.DEBUG:
				print("x",1.0/x)
				print("mean",mean)
				print("std",std)
			probability_pdf = norm.pdf(1.0/x, loc=mean, scale=std) 
			if self.DEBUG:
				print("probability_pdf",probability_pdf)
				probability_pdf_ref = norm.pdf(mean, loc=mean, scale=std)
				print("reference probability_pdf",probability_pdf_ref )


		C1= seed.a/(seed.a+seed.b) * probability_pdf
		# float C2 = seed->b/(seed->a+seed->b) * 1./seed->z_range;
		C2 = seed.b/(seed.a+seed.b)* 1.0/seed.z_range
		# float normalization_constant = C1 + C2;
		normalization_constant = C1 + C2
		# C1 /= normalization_constant;
		C1 = C1/normalization_constant
		# C2 /= normalization_constant;
		C2=C2/normalization_constant
		# float f = C1*(seed->a+1.)/(seed->a+seed->b+1.) + C2*seed->a/(seed->a+seed->b+1.);
		f = C1*(seed.a+1.0)/(seed.a+seed.b+1.0) + C2*seed.a/(seed.a+seed.b+1.0)
		# float e = C1*(seed->a+1.)*(seed->a+2.)/((seed->a+seed->b+1.)*(seed->a+seed->b+2.))
		e = C1*(seed.a+1.0)*(seed.a+2.0)/((seed.a+seed.b+1.0)*(seed.a+seed.b+2.0))+ C2*seed.a*(seed.a+1.0)/((seed.a+seed.b+1.0)*(seed.a+seed.b+2.0))
		#       + C2*seed->a*(seed->a+1.0f)/((seed->a+seed->b+1.0f)*(seed->a+seed->b+2.0f));
		# // update parameters
		# float mu_new = C1*m+C2*seed->mu;
		mu_new = C1*m+C2*seed.mu
		# seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
		seed.sigma2 = C1*(s2 + m*m) + C2*(seed.sigma2 + seed.mu*seed.mu) - mu_new*mu_new
		# seed->mu = mu_new;
		seed.mu = mu_new
		# seed->a = (e-f)/(f-e/f);
		seed.a = (e-f)/(f-e/f)
		# seed->b = seed->a*(1.0f-f)/f;
		seed.b = seed.a*(1.0-f)/f



	def updateSeed(self,x,tau2,seed):
		# void DepthFilter::updateSeed(const float x, const float tau2, Seed* seed)
		# float norm_scale = sqrt(seed->sigma2 + tau2);
		if self.DEBUG:
			print("seed.sigma2",seed.sigma2)
			print("tau2",tau2)
			print("x",x)
		norm_scale=np.sqrt(seed.sigma2+tau2)
		# if(std::isnan(norm_scale))
		if math.isnan(norm_scale):
			return
		# return;
		# boost::math::normal_distribution<float> nd(seed->mu, norm_scale);
		mu, sigma = seed.mu, norm_scale # mean and standard deviation
		s = np.random.normal(mu, sigma, 1)
		# float s2 = 1./(1./seed->sigma2 + 1./tau2);
		s2= 1.0/(1.0/seed.sigma2 + 1.0/tau2)
		# float m = s2*(seed->mu/seed->sigma2 + x/tau2);
		m= s2*(seed.mu/seed.sigma2 + x/tau2)
		# float C1 = seed->a/(seed->a+seed->b) * boost::math::pdf(nd, x);
		probability_pdf=0.0
		INVERSE=True;
		if INVERSE:
			mean = seed.mu
			std = norm_scale
			if self.DEBUG:
				print("x",x)
				print("mean",mean)
				print("std",std)
			probability_pdf = norm.pdf(x, loc=mean, scale=std) 
			if self.DEBUG:
				print("INVERSE probability_pdf",probability_pdf)
				probability_pdf_ref = norm.pdf(mean, loc=mean, scale=std)
				print("INVERSE reference probability_pdf",probability_pdf_ref )
		else:
			mean = 1.0/seed.mu
			std = 1.0/norm_scale
			if self.DEBUG:
				print("x",1.0/x)
				print("mean",mean)
				print("std",std)
			probability_pdf = norm.pdf(1.0/x, loc=mean, scale=std) 
			if self.DEBUG:
				print("probability_pdf",probability_pdf)
				probability_pdf_ref = norm.pdf(mean, loc=mean, scale=std)
				print("reference probability_pdf",probability_pdf_ref )


		C1= seed.a/(seed.a+seed.b) * probability_pdf
		# float C2 = seed->b/(seed->a+seed->b) * 1./seed->z_range;
		C2 = seed.b/(seed.a+seed.b)* 1.0/seed.z_range
		# float normalization_constant = C1 + C2;
		normalization_constant = C1 + C2
		# C1 /= normalization_constant;
		C1 = C1/normalization_constant
		# C2 /= normalization_constant;
		C2=C2/normalization_constant
		# float f = C1*(seed->a+1.)/(seed->a+seed->b+1.) + C2*seed->a/(seed->a+seed->b+1.);
		f = C1*(seed.a+1.0)/(seed.a+seed.b+1.0) + C2*seed.a/(seed.a+seed.b+1.0)
		# float e = C1*(seed->a+1.)*(seed->a+2.)/((seed->a+seed->b+1.)*(seed->a+seed->b+2.))
		e = C1*(seed.a+1.0)*(seed.a+2.0)/((seed.a+seed.b+1.0)*(seed.a+seed.b+2.0))+ C2*seed.a*(seed.a+1.0)/((seed.a+seed.b+1.0)*(seed.a+seed.b+2.0))
		#       + C2*seed->a*(seed->a+1.0f)/((seed->a+seed->b+1.0f)*(seed->a+seed->b+2.0f));
		# // update parameters
		# float mu_new = C1*m+C2*seed->mu;
		mu_new = C1*m+C2*seed.mu
		# seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
		seed.sigma2 = C1*(s2 + m*m) + C2*(seed.sigma2 + seed.mu*seed.mu) - mu_new*mu_new
		# seed->mu = mu_new;
		seed.mu = mu_new
		# seed->a = (e-f)/(f-e/f);
		seed.a = (e-f)/(f-e/f)
		# seed->b = seed->a*(1.0f-f)/f;
		seed.b = seed.a*(1.0-f)/f


	def plot_sphere(self,r,x_,y_,z_):
		u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
		x = 10.0*np.cos(u) * np.sin(v)
		y = 10.0*np.sin(u) * np.sin(v)
		z = 10.0*np.cos(v)
		#self.ax.plot_surface(x+x_, y+y_, z+z_, cmap=plt.cm.YlGnBu_r)
		self.ax.plot_surface(x+x_, y+y_, z+z_, color=[1,0,0])


	def plotting_trajectory(self,uncertainty,direction,plot_both,plot_initial_uncertainty,ISSAC_SIMUALTION,downsample_factor,x_min,x_max,y_min,y_max,z_min,z_max):
		self.plot_both=plot_both
		self.fig,self.ax=plt.subplots()
		self.ax = plt.axes(projection='3d')
		#self.ax.plot3D(self.plotR1,self.plotR2,self.plotR3,'gray')
		self.ax.set_xlabel("x")
		self.ax.set_ylabel("y")
		self.ax.set_zlabel("z")

		self.ax.set_xlim(x_min,x_max)
		self.ax.set_ylim(y_min,y_max)
		self.ax.set_zlim(z_min,z_max)
		# else:
		# 	#self.ax.set_xlim(-450,450)
		# 	#self.ax.set_ylim(-450,450)
		# 	#self.ax.set_zlim(-100,100)	
		#print(len(self.trajectory_quiver_superset))
		count=0.0
		plot_x_line=[]
		plot_y_line=[]
		plot_z_line=[]
		print("length of trajectory_pointcloud_set",len(self.trajectory_pointcloud_set))
		print("length of self.seed_of_points",len(self.seed_of_points))
		#for point in self.trajectory_pointcloud_set:
		#plot point cloud


		if not ISSAC_SIMUALTION:
			sub_sample_factor=1
		else:
			sub_sample_factor=1

		for i in range(0,len(self.trajectory_pointcloud_set),sub_sample_factor):
			if uncertainty:
				uncertainty_after=np.sqrt(self.seed_of_points[i].sigma2)/self.average_sigma

				uncertainty_after=1.0/np.sqrt(self.average_sigma)-1.0/np.sqrt(self.seed_of_points[i].sigma2)
				if(uncertainty_after<=0):
					uncertainty_after=0
				#print("uncertainty_after",uncertainty_after)
				#uncertainty_after=np.sqrt(self.seed_of_points[i].sigma2)
				uncertainty_before=self.trajectory_pointcloud_dir_uncertainty_set[i][3]
				#print("uncertainty_before",uncertainty_before)
				if(plot_initial_uncertainty):
					uncertainty_after=uncertainty_before*3.0
				else:
					uncertainty_after=uncertainty_after*10.0
					if(uncertainty_after>uncertainty_before*3.0):
						uncertainty_after=uncertainty_before*3.0
			if uncertainty==False:
				self.ax.scatter3D(self.trajectory_pointcloud_set[i][0],self.trajectory_pointcloud_set[i][1],self.trajectory_pointcloud_set[i][2],c='green',s=1.5,alpha=1)
			else:
				# if not plot_initial_uncertainty:
				# 	uncertainty_after=uncertainty_after*8
				# else:
				# 	uncertainty_after=uncertainty_after*3
				self.ax.scatter3D(self.trajectory_pointcloud_set[i][0],self.trajectory_pointcloud_set[i][1],self.trajectory_pointcloud_set[i][2],c='green',s=uncertainty_after,alpha=0.15)
				self.ax.scatter3D(self.trajectory_pointcloud_set[i][0],self.trajectory_pointcloud_set[i][1],self.trajectory_pointcloud_set[i][2],c='blue',s=0.5,alpha=1.0)



			if direction==True:	
				quiver=[]
				quiver.append(self.trajectory_pointcloud_set[i][0]);
				quiver.append(self.trajectory_pointcloud_set[i][1]);
				quiver.append(self.trajectory_pointcloud_set[i][2]);

				quiver.append(self.trajectory_pointcloud_dir_uncertainty_set[i][0]);
				quiver.append(self.trajectory_pointcloud_dir_uncertainty_set[i][1]);
				quiver.append(self.trajectory_pointcloud_dir_uncertainty_set[i][2]);			
				self.ax.quiver(quiver[0],quiver[1],quiver[2],quiver[3],quiver[4],quiver[5], color=[0,0,0],length=30, normalize=True)
			


			# if uncertainty==True:
			# 	#plot uncertainty
			# 	f=np.matrix(self.seed_of_points[i].bearing)
			# 	f=f/np.linalg.norm(f)
			# 	t=np.matrix(self.seed_of_points[i].starting_translation)
			# 	z=1.0/self.seed_of_points[i].mu
			# 	p=t+f*z
			# 	x_=p[0,0]
			# 	y_=p[0,1]
			# 	z_=p[0,2]
			# 	#self.ax.scatter3D(x_,y_,z_,c='red',s=1.5)
			# 	#circle= plt.Circle((self.trajectory_pointcloud_set[i][0], self.trajectory_pointcloud_set[i][1]), self.seed_of_points[i].mu/2.0, color='r')
			# 	#self.ax.add_patch(circle)


			# 	if False:
			# 		r=self.seed_of_points[i].sigma2/2.0
			# 		s_x=self.trajectory_pointcloud_set[i][0]
			# 		s_y=self.trajectory_pointcloud_set[i][1]
			# 		s_z=self.trajectory_pointcloud_set[i][2]
			# 		self.plot_sphere(r,s_x,s_y,s_z)



							
		#plot quivers

		if not ISSAC_SIMUALTION:
			quiver_length=0.3
		else:
			quiver_length=3

		
		for set_ in self.trajectory_quiver_superset:
			count+=1.0
			#self.ax.set_prop_cycle(None)
			counter=0
			print("count is ", count)
			for quiver in set_:
				if count==1:
					plot_x_line.append(quiver[0])
					plot_y_line.append(quiver[1])
					plot_z_line.append(quiver[2])
				if ISSAC_SIMUALTION:
					if not counter%downsample_factor==0:
						counter+=1
						continue
				#print(count*1.0/len(set_))
				#if True:
				if self.plot_both:
					if count==2 or count==len(self.trajectory_quiver_superset):#the out put file stores a history of quivers. 
						self.ax.quiver( quiver[0],quiver[1],quiver[2],quiver[3],quiver[4],quiver[5], color=[count*1.0/len(self.trajectory_quiver_superset),0,(len(self.trajectory_quiver_superset)-count)*1.0/len(self.trajectory_quiver_superset)],length=quiver_length*(1.005)**count, normalize=True)
				else:
					if self.view_index=="final" and count==len(self.trajectory_quiver_superset):#the out put file stores a history of quivers. 
						self.ax.quiver( quiver[0],quiver[1],quiver[2],quiver[3],quiver[4],quiver[5], color=[count*1.0/len(self.trajectory_quiver_superset),0,(len(self.trajectory_quiver_superset)-count)*1.0/len(self.trajectory_quiver_superset)],length=quiver_length*(1.005)**count, normalize=True)				
					if count==1 and self.view_index=="initial":#the out put file stores a history of quivers. 
						self.ax.quiver( quiver[0],quiver[1],quiver[2],quiver[3],quiver[4],quiver[5], color=[count*1.0/len(self.trajectory_quiver_superset),0,(len(self.trajectory_quiver_superset)-count)*1.0/len(self.trajectory_quiver_superset)],length=quiver_length*(1.005)**count, normalize=True)
				counter+=1
			#self.ax.scatter3D(path[0],path[1],path[2],c='blue',s=20) //count*count/len(self.trajectory_quiver_superset)
		self.ax.plot(plot_x_line,plot_y_line,plot_z_line)
		#for point in points_list:
		self.ax.view_init(elev=89., azim=270)

		CONTINUITY=False
		COMBINED_CONT_FOV=False
		if CONTINUITY:
			self.ax.set_title("Goal:Rotation Continuity along Trajectory")
		elif COMBINED_CONT_FOV:
			self.ax.set_title("Goal:Combined FOV and Rotation Continuity objectives")
		else:
			self.ax.set_title("Goal:FOV Optimization")
		custom_lines = [Line2D([0], [0], color="blue", lw=4),
		                Line2D([0], [0], color="red", lw=4)]

		#fig, ax = plt.subplots()

		#lines = ax.plot(data)
		self.ax.legend(custom_lines, ['start rotation', 'end rotation'])
		#self.ax2.view_init(elev=10., azim=180.0)
		self.fig.tight_layout()
		#plt.show()
		if CONTINUITY:
			txt="goal:minimize 3D rotation difference between rotation_i and rotation_i+1,\n the start/end waypoint's rotation must be kept unchange"
		elif COMBINED_CONT_FOV:
			txt="Combined FOV and Continuity optimization objectives together"
		else:
			txt="FOV Optimization Only"
		self.fig.text(.5, .05, txt, ha='center')
		plt.show()
		if CONTINUITY:
			self.fig.savefig("./"+self.base_dir+"/"+self.prefix+"trajectory_continuity.pdf",dpi=250)
		elif COMBINED_CONT_FOV:
			self.fig.savefig("./"+self.base_dir+"/"+self.prefix+"combined_fov_continuity_trajectory.pdf",dpi=250)
		else:
			self.fig.savefig("./"+self.base_dir+"/"+self.prefix+"trajectory_fov",dpi=250)
		#self.fig2.savefig("objective_"+self.filename)



if __name__=="__main__":
	uncertainty=False
	traj_downsample_factor=20#trajectory down sample factor
	point_downsample_factor=5
	x_min=-0
	x_max=10
	y_min=-2.5
	y_max=2.5
	z_min=0
	z_max=10



	if len(sys.argv)!=2:
		print("usage need 1 sole argument of base directory")
		sys.exit()

	base_dir = sys.argv[1]
	plt.rcParams['pdf.fonttype'] = 42
	plt.rcParams['ps.fonttype'] = 42
	ISSAC_SIMUALTION=False
	view_index_list=["initial","final"]
	if not ISSAC_SIMUALTION:
		view_type=["test"]
	else:
		view_type=["top","bot","mid","left"]
		#view_type=["top"]




	if True:
		for view_index in view_index_list:
			for view in view_type:
				plot_both=True #plot both initial and final arrows
				ip=None
				if not ISSAC_SIMUALTION:
					ip=simulate_point_uncertainty(view_index+"_issac_"+view,ISSAC_SIMUALTION,base_dir)
				else:
					ip=simulate_point_uncertainty(view_index+"_"+view,ISSAC_SIMUALTION,base_dir)

				if not ISSAC_SIMUALTION: 
					ip.import_from_trajectory_file("./"+base_dir+"/trajectory_history.csv",traj_downsample_factor)
					ip.import_from_xyz_file("./"+base_dir+"/trajectory_pointcloud.csv",point_downsample_factor)
					ip.import_from_pointcloud_dir_file("./"+base_dir+"/trajectory_pointcloud_dir.csv",point_downsample_factor)
				else:
					ip.import_from_trajectory_file("./"+base_dir+"/trajectory_history_"+view+".csv",traj_downsample_factor)
					ip.import_from_xyz_file("./"+base_dir+"/trajectory_pointcloud_"+view+".csv",point_downsample_factor)
					ip.import_from_pointcloud_dir_file("./"+base_dir+"/trajectory_pointcloud_dir_"+view+".csv",point_downsample_factor)
				if uncertainty:
					ip.initialize_point_seed()
				#uncertainty=False
				direction=False
				plot_initial_uncertainty=False

				ip.plotting_trajectory(uncertainty,direction,plot_both,plot_initial_uncertainty,ISSAC_SIMUALTION,1,x_min,x_max,y_min,y_max,z_min,z_max)



				if uncertainty:
					if not ISSAC_SIMUALTION:
						ip.prefix=view_index+"_after_undertainty_plot_"+view
						downsample_factor=1
					else:
						ip.prefix=view_index+"_issac_after_undertainty_plot_"+view
						downsample_factor=1
					ip.simulate_point_uncertainty_at_each_point(view_index,downsample_factor)
					uncertainty=True
					direction=False
					plot_initial_uncertainty=False;
					ip.plotting_trajectory(uncertainty,direction,plot_both,plot_initial_uncertainty,ISSAC_SIMUALTION,1,x_min,x_max,y_min,y_max,z_min,z_max)