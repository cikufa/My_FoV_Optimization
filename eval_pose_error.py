import numpy as np
import statistics
import sys

class eval_pose_error(object):
	def __init__(self,error_filename):
		self.error_filename=error_filename
		self.translation_error_vector=[]
		self.rotation_error_vector=[]
		self.success_rate=None
		self.import_from_error_file()
		self.write_result()

	def import_from_error_file(self):
		success_count=0
		self.file=open(self.error_filename,"r")
		lines=self.file.readlines()
		count=0
		print(self.error_filename,len(lines))
		for line in lines:
			if count==0:
				count+=1
				continue
			line=line.strip("\n")
			splitted_line=line.split(" ")
			#if not splitted_line[-1].isnumeric():
			img_name=splitted_line[0]
			if splitted_line[1] !="nan":
				translation_error=float(splitted_line[1])
				self.translation_error_vector.append(translation_error)
				success_count+=1
			if splitted_line[2] != "nan":
				rotation_error=float(splitted_line[2])
				self.rotation_error_vector.append(rotation_error)
			#call initialize_point_seed() with hard coded or preset directions
			count+=1
		if len(self.rotation_error_vector)>0:
			self.success_rates=float(count)/len(self.rotation_error_vector)
		else:
			self.success_rates=0.0

	def calculate_statistics(self,data):
		# Calculate minimum, maximum, average, and standard deviation
		minimum_value = min(data)
		maximum_value = max(data)
		average_value = statistics.mean(data)
		standard_deviation = statistics.stdev(data)
		return minimum_value, maximum_value, average_value, standard_deviation
	def write_result(self):
		result_file=open("./error_stat.txt","w")
		if(len(self.translation_error_vector)>0 and len(self.rotation_error_vector)):
			t_minimum_value, t_maximum_value, t_average_value, t_standard_deviation=self.calculate_statistics(self.translation_error_vector)
			r_minimum_value, r_maximum_value, r_average_value, r_standard_deviation=self.calculate_statistics(self.rotation_error_vector)
			result_file.write("minimum_value, maximum_value, average_value, standard_deviation;translation_m_row1_rotation_deg_row2\n")
			result_file.write(str(t_minimum_value)+" "+str(t_maximum_value)+" "+ str(t_average_value)+" "+ str(t_standard_deviation)+"\n")
			result_file.write(str(r_minimum_value)+" "+str(r_maximum_value)+" "+ str(r_average_value)+" "+ str(r_standard_deviation)+"\n")
		result_file.write("success_rates "+str(self.success_rates)+"\n")


if __name__=="__main__":
	e=eval_pose_error(sys.argv[1])