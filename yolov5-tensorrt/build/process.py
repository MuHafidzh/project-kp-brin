import cv2
import time
import argparse
import numpy as np

import sys
from pathlib import Path

MODEL = 'm3'

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT)+'/libyolov5-tensorrt.so')
    sys.path.append(str(ROOT)+'/yolov5tensorrt.cpython-311-aarch64-linux-gnu.so')
import yolov5tensorrt

def draw(dets, image):
    color = [
        (255, 51, 153),
        (255, 42, 4),
        (79, 68, 255),
        (255, 0, 189),
        (255, 180, 0),
        (186, 0, 211),
        (0, 192, 38),
        (255, 36, 125),
        (104, 0, 123)
             ]
    
    for det in dets:
        yolov5tensorrt.visualizeDetection(det, image, color[det.classId()], 1.0)

def xyxy2xywh(x): #tak jadi pakai ini
    """Converts nx4 bounding boxes from corners [x1, y1, x2, y2] to center format [x, y, w, h]."""
    y = np.copy(x)
    y[0] = (x[0] + x[2]) / 2  # x center
    y[1] = (x[1] + x[3]) / 2  # y center
    y[2] = x[2] - x[0]  # width
    y[3] = x[3] - x[1]  # height
    return y

def bboxYolo2CxCyWH(x):
    y = np.copy(x)
    y[0] = x[0] + x[2] / 2  # x center
    y[1] = x[1] + x[3] / 2  # y center
    y[2] = x[2]  # width
    y[3] = x[3]  # height
    return y

def from_cam(model, source, vis): #ini gak kepakai kata bagas
    capture = cv2.VideoCapture()

    if not capture.open(source, cv2.CAP_ANY):
        print("failure: could not open capture device")
        return 1

    while True:
        ret, image = capture.read()
        if not ret:
            print("failure: could not read new frames")
            break

        ts = time.perf_counter()

        r, detections = model.detect(image, flags = yolov5tensorrt.DetectorFlag.INPUT_BGR)
        if r != yolov5tensorrt.Result.SUCCESS:
            print("detect() failed:", yolov5tensorrt.result_to_string(r))
            return 1
         
        pred = {
            'id'  :[],
            'name':[],
            'conf':[],
            'xywh':[],
            'det' :[]
        }

        for d in detections:
            pred['id'].append(d.classId())
            pred['name'].append(d.className())
            pred['conf'].append(d.score())
            pred['xywh'].append(xyxy2xywh(d.boundingBox()))

        draw(detections, image)
        duration = time.perf_counter() - ts
        print(f"Inference Time: {duration*1000:.2f}milliseconds\nDetect: {list(set(pred['name']))}")
        if vis:
            cv2.namedWindow("Live")
            cv2.imshow("Live", image)
            cv2.waitKey(1)

    capture.release()
    cv2.destroyAllWindows()

    return 0

def from_cv(model, source): #pakainya yang ini
    image = source
    if image is None:
        print("Failed to load input image")
        return 1

    model.detect(image)
    model.detect(image)

    ts = time.perf_counter()

    r, detections = model.detect(image, flags = yolov5tensorrt.DetectorFlag.INPUT_BGR)
    if r != yolov5tensorrt.Result.SUCCESS:
        print("detect() failed:", yolov5tensorrt.result_to_string(r))
        return 1

    pred = {
            'id'  :[],
            'name':[],
            'conf':[],
            'xywh':[],
            'det' :[]
        }

    for d in detections:
        pred['id'].append(d.classId())
        pred['name'].append(d.className())
        pred['conf'].append(d.score())
        # pred['xywh'].append(xyxy2xywh(d.boundingBox()))
        pred['det'].append(d)
        # bbox = d.boundingBox()
        # print(f"Raw Bounding Box: {bbox}") #setelah di debug dapatlah is x1 y1 w h
        pred['xywh'].append(bboxYolo2CxCyWH(d.boundingBox()))


    duration = time.perf_counter() - ts
    print("Inference Time:", duration*1000, "milliseconds")

    return pred

def init(
    class_file=str(ROOT)+'/classes.txt',
    engine=f'{str(ROOT)}/engine/yolo{str(MODEL)}.engine',
):
    detector = yolov5tensorrt.Detector()
    r = detector.init()
    if r != yolov5tensorrt.Result.SUCCESS:
        print("init() failed:", yolov5tensorrt.result_to_string(r))
        return 1

    r = detector.loadEngine(engine)
    if r != yolov5tensorrt.Result.SUCCESS:
        print("loadEngine() failed:", yolov5tensorrt.result_to_string(r))
        return 1

    if class_file is not None:
        classes = yolov5tensorrt.Classes()
        r = classes.loadFromFile(class_file)
        if r != yolov5tensorrt.Result.SUCCESS:
            print("classes.loadFromFile() failed:", 
                    yolov5tensorrt.result_to_string(r))
            return 1
        detector.setClasses(classes)

    return detector

def parse_opt():
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument('--engine', type=str, default=f'{str(ROOT)}/engine/yolo{str(MODEL)}.engine', help='engine file path')
    parser.add_argument('--source', type=int, required=True, help='camera index or CV Image')
    parser.add_argument('--classes', dest='class_file', type=str, default=str(ROOT)+'/classes.txt', help='list of class names txt array')
    parser.add_argument('--display', action='store_true', help='visualize result')

    opt = parser.parse_args()
    return opt


if __name__ == '__main__':
    arg = parse_opt()
    detector = init(**vars(arg))

    if isinstance(arg.source, np.ndarray):
        res = from_cv(model=detector, source=arg.source)
    
    elif isinstance(arg.source, int):
        res = from_cam(model=detector, source=arg.source, vis=arg.display)
    
    else:
        print('Invalid Source')
