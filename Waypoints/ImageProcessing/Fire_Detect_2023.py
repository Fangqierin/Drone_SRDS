import os
import pathlib
from keras.applications.xception import Xception
from keras.preprocessing import image
from keras.applications.xception import preprocess_input, decode_predictions
import numpy as np
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt 
from keras.preprocessing.image import load_img
from keras.preprocessing.image import img_to_array
import PIL
from PIL import Image
from numpy import expand_dims
from keras.models import load_model
from keras.layers import Conv2D, Input, BatchNormalization, LeakyReLU, ZeroPadding2D, UpSampling2D
from keras.layers.merge import add, concatenate
from keras.models import Model
import cv2
#https://machinelearningmastery.com/how-to-perform-object-detection-with-yolov3-in-keras/
class BoundBox:
    def __init__(self, xmin, ymin, xmax, ymax, objness = None, classes = None):
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        
        self.objness = objness
        self.classes = classes

        self.label = -1
        self.score = -1

    def get_label(self):
        if self.label == -1:
            self.label = np.argmax(self.classes)
        return self.label
    
    def get_score(self):
        if self.score == -1:
            self.score = self.classes[self.get_label()]
        return self.score

def _interval_overlap(interval_a, interval_b):
    x1, x2 = interval_a
    x3, x4 = interval_b

    if x3 < x1:
        if x4 < x1:
            return 0
        else:
            return min(x2,x4) - x1
    else:
        if x2 < x3:
             return 0
        else:
            return min(x2,x4) - x3          

def _sigmoid(x):
    return 1. / (1. + np.exp(-x))

def bbox_iou(box1, box2):
    intersect_w = _interval_overlap([box1.xmin, box1.xmax], [box2.xmin, box2.xmax])
    intersect_h = _interval_overlap([box1.ymin, box1.ymax], [box2.ymin, box2.ymax])
    
    intersect = intersect_w * intersect_h

    w1, h1 = box1.xmax-box1.xmin, box1.ymax-box1.ymin
    w2, h2 = box2.xmax-box2.xmin, box2.ymax-box2.ymin
    
    union = w1*h1 + w2*h2 - intersect
    
    return float(intersect) / union


# def preprocess_input(image, net_h, net_w):
#     new_h, new_w, _ = image.shape
# 
#     # determine the new size of the image
#     if (float(net_w)/new_w) < (float(net_h)/new_h):
#         new_h = (new_h * net_w)/new_w
#         new_w = net_w
#     else:
#         new_w = (new_w * net_h)/new_h
#         new_h = net_h
# 
#     # resize the image to the new size
#     resized = cv2.resize(image[:,:,::-1]/255., (int(new_w), int(new_h)))
# 
#     # embed the image into the standard letter box
#     new_image = np.ones((net_h, net_w, 3)) * 0.5
#     new_image[int((net_h-new_h)//2):int((net_h+new_h)//2), int((net_w-new_w)//2):int((net_w+new_w)//2), :] = resized
#     new_image = np.expand_dims(new_image, 0)
# 
#     return new_image

def decode_netout(netout, anchors, obj_thresh, net_h, net_w):
    grid_h, grid_w = netout.shape[:2]
    nb_box = 3
    netout = netout.reshape((grid_h, grid_w, nb_box, -1))
    nb_class = netout.shape[-1] - 5
    boxes = []
    netout[..., :2]  = _sigmoid(netout[..., :2])
    netout[..., 4:]  = _sigmoid(netout[..., 4:])
    netout[..., 5:]  = netout[..., 4][..., np.newaxis] * netout[..., 5:]
    netout[..., 5:] *= netout[..., 5:] > obj_thresh

    for i in range(grid_h*grid_w):
        row = i / grid_w
        col = i % grid_w
        for b in range(nb_box):
            # 4th element is objectness score
            objectness = netout[int(row)][int(col)][b][4]
            #objectness = netout[..., :4]
            if(objectness.all() <= obj_thresh): continue
            # first 4 elements are x, y, w, and h
            x, y, w, h = netout[int(row)][int(col)][b][:4]
            x = (col + x) / grid_w # center position, unit: image width
            y = (row + y) / grid_h # center position, unit: image height
            w = anchors[2 * b + 0] * np.exp(w) / net_w # unit: image width
            h = anchors[2 * b + 1] * np.exp(h) / net_h # unit: image height  
            # last elements are class probabilities
            classes = netout[int(row)][col][b][5:]
            box = BoundBox(x-w/2, y-h/2, x+w/2, y+h/2, objectness, classes)
            #box = BoundBox(x-w/2, y-h/2, x+w/2, y+h/2, None, classes)
            boxes.append(box)

    return boxes

def correct_yolo_boxes(boxes, image_h, image_w, net_h, net_w):
    if (float(net_w)/image_w) < (float(net_h)/image_h):
        new_w = net_w
        new_h = (image_h*net_w)/image_w
    else:
        new_h = net_w
        new_w = (image_w*net_h)/image_h
        
    for i in range(len(boxes)):
        x_offset, x_scale = (net_w - new_w)/2./net_w, float(new_w)/net_w
        y_offset, y_scale = (net_h - new_h)/2./net_h, float(new_h)/net_h
        
        boxes[i].xmin = int((boxes[i].xmin - x_offset) / x_scale * image_w)
        boxes[i].xmax = int((boxes[i].xmax - x_offset) / x_scale * image_w)
        boxes[i].ymin = int((boxes[i].ymin - y_offset) / y_scale * image_h)
        boxes[i].ymax = int((boxes[i].ymax - y_offset) / y_scale * image_h)
        
def do_nms(boxes, nms_thresh):
    if len(boxes) > 0:
        nb_class = len(boxes[0].classes)
    else:
        return
        
    for c in range(nb_class):
        sorted_indices = np.argsort([-box.classes[c] for box in boxes])

        for i in range(len(sorted_indices)):
            index_i = sorted_indices[i]

            if boxes[index_i].classes[c] == 0: continue

            for j in range(i+1, len(sorted_indices)):
                index_j = sorted_indices[j]

                if bbox_iou(boxes[index_i], boxes[index_j]) >= nms_thresh:
                    boxes[index_j].classes[c] = 0  

# def _main_(args):
#     weights_path = args.weights
#     image_path   = args.image
#     # set some parameters
#     net_h, net_w = 416, 416
#     obj_thresh, nms_thresh = 0.5, 0.45
#     anchors = [[116,90,  156,198,  373,326],  [30,61, 62,45,  59,119], [10,13,  16,30,  33,23]]
#     labels = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", \
#               "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", \
#               "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", \
#               "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", \
#               "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", \
#               "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", \
#               "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", \
#               "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse", \
#               "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", \
#               "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
# 
#     # make the yolov3 model to predict 80 classes on COCO
#     yolov3 = make_yolov3_model()
# 
#     # load the weights trained on COCO into the model
#     weight_reader = WeightReader(weights_path)
#     weight_reader.load_weights(yolov3)
# 
#     # preprocess the image
#     image = cv2.imread(image_path)
#     image_h, image_w, _ = image.shape
#     new_image = preprocess_input(image, net_h, net_w)
# 
#     # run the prediction
#     yolos = yolov3.predict(new_image)
#     boxes = []
# 
#     for i in range(len(yolos)):
#         # decode the output of the network
#         boxes += decode_netout(yolos[i][0], anchors[i], obj_thresh, nms_thresh, net_h, net_w)
# 
#     # correct the sizes of the bounding boxes
#     correct_yolo_boxes(boxes, image_h, image_w, net_h, net_w)
# 
#     # suppress non-maximal boxes
#     do_nms(boxes, nms_thresh)     
# 
#     # draw bounding boxes on the image using labels
#     draw_boxes(image, boxes, labels, obj_thresh) 
#  
#     # write the image with bounding boxes to file
#     cv2.imwrite(image_path[:-4] + '_detected' + image_path[-4:], (image).astype('uint8')) 
def load_image_pixels(filename, shape):
    # load the image to get its shape
    image = load_img(filename)
    width, height = image.size
    # load the image with the required size
    image = load_img(filename, target_size=shape)
    # convert to numpy array
    image = img_to_array(image)
    # scale pixel values to [0, 1]
    image = image.astype('float32')
    image /= 255.0
    # add a dimension so that we have one sample
    image = expand_dims(image, 0)
    return image, width, height

def get_boxes(boxes, labels, thresh):
    v_boxes, v_labels, v_scores = list(), list(), list()
    # enumerate all boxes
    for box in boxes:
    # enumerate all possible labels
        for i in range(len(labels)):
        # check if the threshold for this label is high enough
            if box.classes[i] > thresh:
                v_boxes.append(box)
                v_labels.append(labels[i])
                v_scores.append(box.classes[i]*100)
 # don't break, many labels may trigger for one box
    return v_boxes, v_labels, v_scores

def draw_boxes(filename, v_boxes, v_labels, v_scores,outputfolder):
    # load the image
    data = plt.imread(filename)
    # plot the image
    #print(data.shape)
    plt.imshow(data)
    # get the context for drawing boxes
    ax = plt.gca()
    #plt.axis('off')
    # plot each box
    name=(str(filename).split('/')[-1]).split('.')[0]
#     if len(v_boxes)==0:
#         print(f"No Fire")
    for i in range(len(v_boxes)):
        print(f"detect fire", v_labels[i], v_scores[i],v_boxes[i])
        box = v_boxes[i]
        # get coordinates
        y1, x1, y2, x2 = box.ymin, box.xmin, box.ymax, box.xmax
        # calculate width and height of the box
        width, height = x2 - x1, y2 - y1
        # create the shape
        rect = Rectangle((x1, y1), width, height, fill=False, color='blue')
        # draw the box
        ax.add_patch(rect)
        # draw text and score in top left corner
        label = "%s (%.3f)" % (v_labels[i], v_scores[i])
        plt.text((x2+x1)/2, (y2+y1)/2, label, color='blue')
        name=name+f"({x1},{y1},{x2},{y2})"
    plt.savefig(f"{outputfolder}{name}.jpg",bbox_inches='tight',pad_inches = 0)
    
def fire_detect(filename,outputfolder):
    execution_path = os.getcwd()
    modelpath=os.path.join(execution_path, "detection_model-ex-33--loss-4.97.h5")
    model=load_model(modelpath)
    input_h,input_w=416,416
    class_threshold=0.05
    boxes = list()
    anchors=[[129,208,183,155,198,259],[73,140,109,80,119,131],[22,31,31,57,54,80]]
    image, image_w, image_h = load_image_pixels(filename, (input_w, input_h))
    yhat=model.predict(image)
    for i in range(len(yhat)):
        boxes += decode_netout(yhat[i][0], anchors[i], class_threshold, input_h, input_w)
    correct_yolo_boxes(boxes, image_h, image_w, input_h, input_w)
    do_nms(boxes, 0.5)
    v_boxes, v_labels, v_scores = get_boxes(boxes, ['fire'], class_threshold)
    draw_boxes(images, v_boxes, v_labels, v_scores,outputfolder)


if __name__ == '__main__':
    Paths_to_dir=pathlib.Path('./images/Mockupfire')
    outputfolder='./images/MockupfireResult/'
    Test_image_paths=sorted(list(Paths_to_dir.glob("*.j*")))
    for images in Test_image_paths:
        fire_detect(images,outputfolder)
        
        
        




