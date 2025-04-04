# get yolov3.weights from https://pjreddie.com/darknet/yolo/
###  wget https://pjreddie.com/media/files/yolov3.weights  
#git clone https://github.com/pjreddie/darknet
##   find darknet/data/coco.names
##   find darknet/cfg/yolov3.cfg  
import cv2
import numpy as np
import time
import os
import sys




class human_detector:
    def __init__(self,workspace="~",debug=True):
        self.initialize_model()
        self.debug=debug
        self.workspace=workspace

        # Specify the path of the directory you want to create
        self.image_output_directory_path = self.workspace+"/images_with_detection/"
        self.image_input_directory_path=self.workspace+"/images/"
        self.initialize_output_path(self.image_output_directory_path)
        self.image_files=self.get_files(self.image_input_directory_path,'.png')
        self.output_detection_file=open(self.workspace+"/detection_result.txt","w")
        self.detection_list=[]
        for image in self.image_files:
            self.detect_human(image)
        self.output_detection_file_sum=open(self.workspace+"/detection_result_sum.txt","w")
        self.output_detection_file_sum.write(str(sum(self.detection_list))+'\n')
        self.output_detection_file_sum.close()
        self.output_detection_file.close()


    def initialize_model(self,yolo_weight_path="/media/chen/T7_cam_2_3/FOV_DIR/FOV_Optimization_On_Manifold_/Detection/yolov3.weights",yolo_cfg_path="/media/chen/T7_cam_2_3/FOV_DIR/FOV_Optimization_On_Manifold_/Detection/yolov3.cfg",coco_name_path="/media/chen/T7_cam_2_3/FOV_DIR/FOV_Optimization_On_Manifold_/Detection/coco.names",debug=False):
        # Load YOLO model
        self.net = cv2.dnn.readNet(yolo_weight_path, yolo_cfg_path)
        classes = []
        with open(coco_name_path, "r") as f:
            classes = [line.strip() for line in f.readlines()]
        layer_names = self.net.getLayerNames()
        print(layer_names)
        for i in self.net.getUnconnectedOutLayers():
            print(i)
            print(layer_names[i - 1])
        #output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
        #time.sleep(1000)
        self.output_layers=[layer_names[199],layer_names[226],layer_names[253]] #hard coded 
        

    def detect_human(self,image_path="human.jpg"):
        # Load image
        image = cv2.imread(self.image_input_directory_path+image_path)
        height, width, channels = image.shape
        # Detect objects using YOLO
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        print("forward finished")
        # Process detections
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and class_id == 0:  # Class ID 0 is for humans
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-maximum suppression to eliminate redundant overlapping boxes
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        # Draw bounding boxes around detected humans
        print("detected",len(indexes),"humans")
        if(self.debug):
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Display result
            cv2.imwrite(self.image_output_directory_path+image_path, image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            self.output_detection_file.write(str(len(indexes))+'\n')
            self.detection_list.append(len(indexes))
    def get_files(self,directory_path,post_fix):
        # Get a list of all files in the directory
        files_in_directory = os.listdir(directory_path)

        # Use list comprehension to filter files ending with '.png'
        files = [file for file in files_in_directory if file.endswith(post_fix)]
        return files

    def initialize_output_path(self,directory_path):
        # Check if the directory already exists
        if not os.path.exists(directory_path):
            # Create the directory if it doesn't exist
            os.makedirs(directory_path)
            print("Directory created successfully.")
        else:
            print("Directory already exists.")
if __name__=="__main__":
    # Check if at least one argument is provided
    run_directory=None
    if len(sys.argv) == 2:
        # Get the first argument (index 0 is the script name)
        run_directory = sys.argv[1]
        print("Run Directory:", run_directory)
    else:
        print("syntax: python cv2_detect.py <run_directory>")

    hd=human_detector(run_directory)