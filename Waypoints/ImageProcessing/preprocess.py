import cv2
import os
import pathlib
def saveimages(filename, outputfolder):
    filename = str(filename)
    cap = cv2.VideoCapture(filename)

    frame_count = 0

    name = (str(filename).split('/')[-1]).split('.')[0]

    while cap.isOpened():
        ret, frame = cap.read()
        # if not ret or frame_count >= test_size:
        if not ret or frame_count>=10:
            break
        # create folder
        save_folder = outputfolder+f"{name}/"
        if not os.path.exists(save_folder):
            os.makedirs(save_folder)
        save_name = save_folder + f"{frame_count}.jpg"
        cv2.imwrite(save_name, frame)
        frame_count += 1

    cap.release()
    cv2.destroyAllWindows()




if __name__ == '__main__':
    execution_path = os.getcwd()
    Paths_to_dir = pathlib.Path('./images/Mockupfire/video')
    outputfolder = './images/video2images/'
    extensions = (
        '*.jpg',
        '*.jpeg',
        '*.png',
        '*.gif', '*.bmp',
        '*.mp4',
        '*.avi', '*.wmv', '*.mov', '*.mkv',
        '*.MOV'
    )
    Test_image_paths = []
    for ext in extensions:
        Test_image_paths.extend(sorted(list(Paths_to_dir.glob(ext))))
    for filename in Test_image_paths:
        saveimages(filename, outputfolder)