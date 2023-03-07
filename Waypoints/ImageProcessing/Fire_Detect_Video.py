import os
import pathlib
import numpy as np
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from keras.utils.image_utils import load_img, img_to_array
from numpy import expand_dims
from keras.models import load_model
from ResultWriter import ResultWriter
import cv2


# https://machinelearningmastery.com/how-to-perform-object-detection-with-yolov3-in-keras/
class BoundBox:
    def __init__(self, xmin, ymin, xmax, ymax, objness=None, classes=None):
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
            return min(x2, x4) - x1
    else:
        if x2 < x3:
            return 0
        else:
            return min(x2, x4) - x3


def _sigmoid(x):
    return 1. / (1. + np.exp(-x))


def bbox_iou(box1, box2):
    intersect_w = _interval_overlap([box1.xmin, box1.xmax], [box2.xmin, box2.xmax])
    intersect_h = _interval_overlap([box1.ymin, box1.ymax], [box2.ymin, box2.ymax])

    intersect = intersect_w * intersect_h

    w1, h1 = box1.xmax - box1.xmin, box1.ymax - box1.ymin
    w2, h2 = box2.xmax - box2.xmin, box2.ymax - box2.ymin

    union = w1 * h1 + w2 * h2 - intersect

    return float(intersect) / union


def decode_netout(netout, anchors, obj_thresh, net_h, net_w):
    grid_h, grid_w = netout.shape[:2]
    nb_box = 3
    netout = netout.reshape((grid_h, grid_w, nb_box, -1))
    nb_class = netout.shape[-1] - 5
    boxes = []
    netout[..., :2] = _sigmoid(netout[..., :2])
    netout[..., 4:] = _sigmoid(netout[..., 4:])
    netout[..., 5:] = netout[..., 4][..., np.newaxis] * netout[..., 5:]
    netout[..., 5:] *= netout[..., 5:] > obj_thresh

    for i in range(grid_h * grid_w):
        row = i / grid_w
        col = i % grid_w
        for b in range(nb_box):
            # 4th element is objectness score
            objectness = netout[int(row)][int(col)][b][4]
            # objectness = netout[..., :4]
            if (objectness.all() <= obj_thresh): continue
            # first 4 elements are x, y, w, and h
            x, y, w, h = netout[int(row)][int(col)][b][:4]
            x = (col + x) / grid_w  # center position, unit: image width
            y = (row + y) / grid_h  # center position, unit: image height
            w = anchors[2 * b + 0] * np.exp(w) / net_w  # unit: image width
            h = anchors[2 * b + 1] * np.exp(h) / net_h  # unit: image height
            # last elements are class probabilities
            classes = netout[int(row)][col][b][5:]
            box = BoundBox(x - w / 2, y - h / 2, x + w / 2, y + h / 2, objectness, classes)
            # box = BoundBox(x-w/2, y-h/2, x+w/2, y+h/2, None, classes)
            boxes.append(box)

    return boxes


def correct_yolo_boxes(boxes, image_h, image_w, net_h, net_w):
    if (float(net_w) / image_w) < (float(net_h) / image_h):
        new_w = net_w
        new_h = (image_h * net_w) / image_w
    else:
        new_h = net_w
        new_w = (image_w * net_h) / image_h

    for i in range(len(boxes)):
        x_offset, x_scale = (net_w - new_w) / 2. / net_w, float(new_w) / net_w
        y_offset, y_scale = (net_h - new_h) / 2. / net_h, float(new_h) / net_h

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

            for j in range(i + 1, len(sorted_indices)):
                index_j = sorted_indices[j]

                if bbox_iou(boxes[index_i], boxes[index_j]) >= nms_thresh:
                    boxes[index_j].classes[c] = 0

def load_image_pixels(input_image, shape):
    # load the image to get its shape
    # load the image with the required size
    # image = load_img(filename, target_size=shape)
    # convert to numpy array
    # image = img_to_array(image)
    image = cv2.resize(input_image, shape)
    np.asarray(image)
    # scale pixel values to [0, 1]
    image = image.astype('float32')
    image /= 255.0
    # add a dimension so that we have one sample
    image = expand_dims(image, 0)
    return image


def detect_from_image(filename, outputfolder, model):
    input_image = cv2.imread(str(filename))
    print("Original input size: ", input_image.shape)
    input_image_rgb = bgr_to_rgb(input_image)
    fire_detect(filename, input_image_rgb, outputfolder, 0, model)


def detect_from_video(filename, outputfolder, model, rw):
    filename = str(filename)
    cap = cv2.VideoCapture(filename)

    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_num = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    test_size = 300
    fire_count = 0
    frame_count = 0


    name = (str(filename).split('/')[-1]).split('.')[0]

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(outputfolder + f"{name}_result.avi", fourcc, fps, (width, height), isColor=True)

    while cap.isOpened():
        ret, frame = cap.read()
        # if not ret or frame_count >= test_size:
        if not ret:
            break
        processed_frame, has_fire = fire_detect(filename, frame, outputfolder, 1, model)
        if has_fire:
            fire_count += 1
        processed_frame = rgba_to_rgb(processed_frame)
        cv2.imshow('frame', processed_frame)
        # reshape to fit video output
        processed_frame = cv2.resize(processed_frame, (width, height))
        print(processed_frame.shape)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # exit by pressing 'q'
            break
        out.write(processed_frame)
        frame_count += 1
        print("%d/%d frames for %s processed"%(frame_count, frame_num, name))

    cap.release()
    out.release()
    cv2.destroyAllWindows()
    distance = name.split('_')[1]
    accuracy = format(fire_count / test_size, '.5f')
    rw.write_row(distance, accuracy)


def bgr_to_rgb(brg_img):
    b, g, r = cv2.split(brg_img)
    img_rgb = cv2.merge([r, g, b])
    return img_rgb


def rgb_to_bgr(rgb_img):
    r, g, b, t = cv2.split(rgb_img)
    img_bgr = cv2.merge([b, g, r])
    return img_bgr


def rgba_to_rgb(rgba_img):
    r, g, b, a = cv2.split(rgba_img)
    img_rgb = cv2.merge([r, g, b])
    return img_rgb


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
                v_scores.append(box.classes[i] * 100)
    # don't break, many labels may trigger for one box
    return v_boxes, v_labels, v_scores


def draw_boxes(filename, input_image, v_boxes, v_labels, v_scores, outputfolder, type):
    plt.imshow(input_image)
    print("drawbox input size: ", input_image.shape)
    has_fire = True

    ax = plt.gca()
    # plt.axis('off')
    # plot each box
    name = (str(filename).split('/')[-1]).split('.')[0]
    if len(v_boxes)==0:
        has_fire = False
    for i in range(len(v_boxes)):
        print(f"detect fire", v_labels[i], v_scores[i], v_boxes[i])
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
        plt.text((x2 + x1) / 2, (y2 + y1) / 2, label, color='blue')
        name = name + f"({x1},{y1},{x2},{y2})"
        # plt.axis('off')
    if type == 0:
        plt.show()
        plt.savefig(f"{outputfolder}{name}.jpg", bbox_inches='tight', pad_inches=0)
        plt.close()
    else:
        # render the figure
        fig = plt.gcf()
        canvas = FigureCanvas(fig)
        canvas.draw()
        # get the pixel buffer
        buf = canvas.buffer_rgba()
        # convert to a NumPy array
        input_image = np.asarray(buf)
        # input_image = input_image.reshape((width, height, 4))
        # input_image = cv2.resize(input_image, (width, height))
        print("After boxing size: ", input_image.shape)
        # close the figure to free up memory
        plt.close(fig)
        return input_image, has_fire


def fire_detect(filename, input_image, outputfolder, type, model):
    input_h, input_w = 416, 416
    # class_threshold = 0.05
    class_threshold = 0.1
    boxes = list()
    anchors = [[129, 208, 183, 155, 198, 259], [73, 140, 109, 80, 119, 131], [22, 31, 31, 57, 54, 80]]
    size = input_image.shape

    image_w = size[1]
    image_h = size[0]
    image = load_image_pixels(input_image, (input_w, input_h))
    yhat = model.predict(image)
    for i in range(len(yhat)):
        boxes += decode_netout(yhat[i][0], anchors[i], class_threshold, input_h, input_w)
    correct_yolo_boxes(boxes, image_h, image_w, input_h, input_w)
    do_nms(boxes, 0.5)
    v_boxes, v_labels, v_scores = get_boxes(boxes, ['fire'], class_threshold)
    # im_array = np.array(image)
    if type == 0:
        draw_boxes(filename, input_image, v_boxes, v_labels, v_scores, outputfolder, type)
    else:
        frame, has_fire = draw_boxes(filename, input_image, v_boxes, v_labels, v_scores, outputfolder, type)
        return frame, has_fire


if __name__ == '__main__':
    execution_path = os.getcwd()
    modelpath = os.path.join(execution_path, "detection_model-ex-33--loss-4.97.h5")
    model = load_model(modelpath)
    # Paths_to_dir = pathlib.Path('./images/Mockupfire/test')
    Paths_to_dir = pathlib.Path('./images/video2images/fire_5m')
    outputfolder = './images/MockupfireResult/'
    extensions = (
        '*.jpg',
        '*.jpeg',
        '*.png',
        '*.gif', '*.bmp',
        '*.mp4',
        '*.avi', '*.wmv', '*.mov', '*.mkv',
        '*.MOV'
    )
    # Test_image_paths=sorted(list(Paths_to_dir.glob("*.j*")))
    # Test_image_paths = sorted(list(Paths_to_dir.glob(extensions)))
    # rw = ResultWriter("result.csv")
    Test_image_paths = []
    for ext in extensions:
        Test_image_paths.extend(sorted(list(Paths_to_dir.glob(ext))))
    for filename in Test_image_paths:
        detect_from_image(filename, outputfolder, model)
        # detect_from_video(filename, outputfolder, model, rw)
    # rw.write_to_csv()






