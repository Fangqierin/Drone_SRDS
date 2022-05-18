import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

import pathlib
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image
from IPython.display import display
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
# patch tf1 into `utils.ops`
utils_ops.tf = tf.compat.v1

# Patch the location of gfile
tf.gfile = tf.io.gfile

def load_model(model_name):
    base_url = 'http://download.tensorflow.org/models/object_detection/'
    model_file = model_name + '.tar.gz'
    model_dir = tf.keras.utils.get_file(
      fname=model_name, 
      origin=base_url + model_file,
      untar=True)
    
    model_dir = pathlib.Path(model_dir)/"saved_model"
    
    model = tf.saved_model.load(str(model_dir))
    model = model.signatures['serving_default']
    
    return model

def run_inference_for_single_image(model, image):
    image = np.asarray(image)
    # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
    input_tensor = tf.convert_to_tensor(image)
    # The model expects a batch of images, so add an axis with `tf.newaxis`.
    input_tensor = input_tensor[tf.newaxis,...]
    
    # Run inference
    output_dict = model(input_tensor)
    # All outputs are batches tensors.
    # Convert to numpy arrays, and take index [0] to remove the batch dimension.
    # We're only interested in the first num_detections.
    num_detections = int(output_dict.pop('num_detections'))
    output_dict = {key:value[0, :num_detections].numpy() 
                   for key,value in output_dict.items()}
    output_dict['num_detections'] = num_detections
    
    # detection_classes should be ints.
    output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)
    
    # Handle models with masks:
    if 'detection_masks' in output_dict:
    # Reframe the the bbox mask to the image size.
        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                  output_dict['detection_masks'], output_dict['detection_boxes'],
                   image.shape[0], image.shape[1])      
        detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,
                                           tf.uint8)
        output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()
      
    return output_dict

def show_inference(model, image_path,output_path):
  # the array based representation of the image will be used later in order to prepare the
  # result image with boxes and labels on it.
    name=str(image_path).split('/')[-1] 
    result_name=f"{output_path}/result_{name}"
    image_np = np.array(Image.open(image_path))
    # Actual detection.
    output_dict = run_inference_for_single_image(model, image_np)
    ###########
    output_dict2={}
    for i in output_dict.keys():
        print(f"ss",i,output_dict.get(i))
    if output_dict.get('num_detections')>0:
        if 1 in output_dict.get('detection_classes'):
            tmp=output_dict.get('detection_classes')
            human=[i for i in range(len(tmp)) if tmp[i]==1]
            #print (f"why",output_dict.get('detection_boxes').shape,[output_dict.get('detection_boxes')[i] for i in human].as_list())
            output_dict2['detection_boxes']= tf.constant([output_dict.get('detection_boxes')[i] for i in human]).numpy()
            output_dict2['detection_scores']=tf.constant([output_dict.get('detection_scores')[i] for i in human]).numpy()
            output_dict2['detection_classes']=tf.constant([output_dict.get('detection_classes')[i] for i in human]).numpy()
            output_dict2['num_detections']=len(human)
#             for i in output_dict.keys():
#                 print(f"see out_2", i,output_dict2.get(i))
#     # Visualization of the results of a detection.
            output_dict=output_dict2
   
    vis_util.visualize_boxes_and_labels_on_image_array(
        image_np,
        output_dict['detection_boxes'],
        output_dict['detection_classes'],
        output_dict['detection_scores'],
        category_index,
        instance_masks=output_dict.get('detection_masks_reframed', None),
        use_normalized_coordinates=True,
        line_thickness=8,
        min_score_thresh=0.3
    )
    
    final=Image.fromarray(image_np)
    final.save(result_name)
    display(final)

model_name = "mask_rcnn_inception_resnet_v2_atrous_coco_2018_01_28"
s2="ssd_mobilenet_v1_0.75_depth_300x300_coco14_sync_2018_07_03"
s3="faster_rcnn_inception_v2_coco_2018_01_28"
masking_model = load_model("faster_rcnn_inception_v2_coco_2018_01_28")

PATH_TO_TEST_IMAGES_DIR = pathlib.Path('object_detection/test_images/close/')
output_path='object_detection/test_images/close_result'
TEST_IMAGE_PATHS = sorted(list(PATH_TO_TEST_IMAGES_DIR.glob("*.jpeg")))
names=[str(i).split('/')[-1] for i in TEST_IMAGE_PATHS]
result_names=[f"result_{i}" for i in names]
print(result_names)
PATH_TO_LABELS = 'object_detection/data/mscoco_label_map.pbtxt'
category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)
for image_path in TEST_IMAGE_PATHS:
    show_inference(masking_model, image_path,output_path)


