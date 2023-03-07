from imageai.Detection.Custom import CustomObjectDetection, CustomVideoObjectDetection
import os
import fire_net as fn
execution_path = os.getcwd()
import pathlib
from PIL import Image
from tensorflow import keras
#import PyTorch 

def detect_from_image(image_path,output_path):
    print(image_path)
    detector = CustomObjectDetection()
    detector.setModelTypeAsYOLOv3()
#     detector.setModelPath(detection_model_path=os.path.join(execution_path, "detection_model-ex-33--loss-4.97.h5"))
#     detector.setJsonPath(configuration_json=os.path.join(execution_path, "detection_config.json"))
    #print(os.path.join(execution_path, "detection_model-ex-33--loss-4.97.h5"))
    #model = keras.models.load_model(os.path.join(execution_path, "detection_model-ex-33--loss-4.97.h5"))
    #keras.models.save_model(model,"/Users/fangqiliu/Desktop/FireNET-master/")
    #model.save("/Users/fangqiliu/Desktop/FireNET-master/")
    #detector.setModelPath(os.path.join(execution_path, "detection_model-ex-33--loss-4.97.pt/saved_model.pb"))

    detector.setModelPath(os.path.join(execution_path, "detection_model-ex-33--loss-4.97.h5"))
    #model = keras.models.load_model("detection_model-ex-33--loss-4.97.h5")

    detector.setJsonPath(os.path.join(execution_path, "detection_config.json"))
    detector.loadModel()
    name=str(image_path).split('/')[-1]
    result_name=f"{output_path}/ressult_{name}"
    detections = detector.detectObjectsFromImage(input_image=os.path.join(execution_path, image_path),
                                                 output_image_path=os.path.join(execution_path, result_name),
                                                 minimum_percentage_probability=5)

    for detection in detections:
        print(detection["name"], " : ", detection["percentage_probability"], " : ", detection["box_points"])
        
def detect_from_video():
    detector = CustomVideoObjectDetection()
    detector.setModelTypeAsYOLOv3()
    detector.setModelPath(detection_model_path=os.path.join(execution_path, "detection_model-ex-33--loss-4.97.h5"))
    detector.setJsonPath(configuration_json=os.path.join(execution_path, "detection_config.json"))
    detector.loadModel()

    detected_video_path = detector.detectObjectsFromVideo(input_file_path=os.path.join(execution_path, "video1.mp4"), frames_per_second=30, output_file_path=os.path.join(execution_path, "video1-detected"), minimum_percentage_probability=40, log_progress=True )

def show_reference(image_path):
    image=Image.open(image_path)
    #output_dict=run_reference_for_single_image(model,image)


Paths_to_dir=pathlib.Path('./images/close')
Test_image_paths=sorted(list(Paths_to_dir.glob("*.j*")))
output_path="images/close_result/"
for images in Test_image_paths:
    print(images)
    detect_from_image(images,output_path)
    
    
    
    

#print(Test_image_paths)
#detect_from_image()
#detect_from_video()