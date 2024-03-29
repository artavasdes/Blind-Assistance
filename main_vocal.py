# Original version that uses speech to position objects
from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import os
import asyncio
from utils import speech, gmaps
import multiprocessing

'''
Spatial Tiny-yolo example
  Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
  Can be used for tiny-yolo-v3 or tiny-yolo-v4 networks
'''

#Make sure to change anchors according to json

if not os.path.exists("temp"):
    os.makedirs("temp")

#YoloV7-tiny
nnBlobPath = str((Path(__file__).parent / Path('models/yolov7tiny-640x480.blob')).resolve().absolute())

if not Path(nnBlobPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

labelMap = [
    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
    "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"
]

#Out of 10
#TODO Create tuples as values, first val for normal and second for urgent weight
detection_weights = {
    "person": 5,
    "bicycle": 7,    
    "car": 10,           
    "motorbike": 9,       
    "bus": 10,           
    "train": 10,
    "truck": 10,          
    "fire hydrant": 3, 
    "parking meter": 3, 
    "bench": 3,
    "dog": 4, 
}

syncNN = True

#Boolean for setting if audio multiprocess is done
audio_alive = False

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
nnNetworkOut = pipeline.create(dai.node.XLinkOut)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutDepth.setStreamName("depth")
nnNetworkOut.setStreamName("nnNetwork")

# Properties
#camRgb.setPreviewSize(416, 416)
#608 812
camRgb.setPreviewSize(640, 480)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# setting node configs
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Align depth map to the perspective of RGB camera, on which inference is done
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# Yolo specific parameters
spatialDetectionNetwork.setNumClasses(80)
spatialDetectionNetwork.setCoordinateSize(4)
spatialDetectionNetwork.setAnchors([12.0, 16.0, 19.0, 36.0, 40.0, 28.0, 36.0, 75.0, 76.0, 55.0, 72.0, 146.0, 142.0, 110.0, 192.0, 243.0, 459.0, 401.0])
spatialDetectionNetwork.setAnchorMasks({"side80": [0, 1, 2],  "side40": [3, 4, 5], "side20": [6, 7, 8]})

spatialDetectionNetwork.setIouThreshold(0.5)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
else:
    camRgb.preview.link(xoutRgb.input)

spatialDetectionNetwork.out.link(xoutNN.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)

def main():
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        calibData = device.readCalibration()

        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=4, blocking=False);

        startTime = time.monotonic()
        counter = 0
        fps = 0
        color = (255, 255, 255)
        printOutputLayersOnce = True

        loop_iter = 0
        while True:
            inPreview = previewQueue.get()
            inDet = detectionNNQueue.get()
            depth = depthQueue.get()
            inNN = networkQueue.get()

            if printOutputLayersOnce:
                toPrint = 'Output layer names:'
                for ten in inNN.getAllLayerNames():
                    toPrint = f'{toPrint} {ten},'
                print(toPrint)
                printOutputLayersOnce = False;

            frame = inPreview.getCvFrame()
            depthFrame = depth.getFrame() # depthFrame values are in millimeters

            depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            depthFrameColor = cv2.equalizeHist(depthFrameColor)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

            counter+=1
            current_time = time.monotonic()
            if (current_time - startTime) > 1 :
                fps = counter / (current_time - startTime)
                counter = 0
                startTime = current_time

            detections = inDet.detections

            #List for tracking all objects detected in each frame
            detected_frame_objects = []

            # If the frame is available, draw bounding boxes on it and show the frame
            height = frame.shape[0]
            width  = frame.shape[1]
            for detection in detections:
                roiData = detection.boundingBoxMapping
                roi = roiData.roi
                roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                topLeft = roi.topLeft()
                bottomRight = roi.bottomRight()
                xmin = int(topLeft.x)
                ymin = int(topLeft.y)
                xmax = int(bottomRight.x)
                ymax = int(bottomRight.y)
                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)

                # Denormalize bounding box
                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)
                try:
                    label = labelMap[detection.label]
                    detected_frame_objects.append(label)
                except:
                    label = detection.label
                cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

            #HFOV: 68.7938003540039
            
            #TODO, make a center direction for low degrees !!!
            #TODO, create a dictionary of labels and values assigning weights to different objects and which one takes priority for alerting the user
            #TODO find movement of object by how fast it moves in pixels, ex: if car is moving fast then alert immediatley
            
                
            #Searches for highest priority object detected
            highest_priority_obj = (None, 0)
            for label in detected_frame_objects:
                if label in detection_weights and detection_weights[label] > highest_priority_obj[1]:
                    highest_priority_obj = (label, detection_weights[label]) 


            if (highest_priority_obj[0] is not None and loop_iter == 0):
                object_x_average = ((xmin + xmin)/2)/width
                direction = (object_x_average * calibData.getFov(dai.CameraBoardSocket.RGB)) - (calibData.getFov(dai.CameraBoardSocket.RGB)/2)
                if direction < -15:
                    body_rel_direction = "left"
                elif direction > 15:
                    body_rel_direction = "right"
                else:
                    body_rel_direction = "center"
                audio = "text_to_speech.mp3"
                speech.direction_distance(audio, round((detection.spatialCoordinates.z/1000),2), abs(round(direction)), highest_priority_obj[0], body_rel_direction)
                
                #speech.speed_setter(audio, 2)

                audio = multiprocessing.Process(target=speech.play_audio, args=((audio),))
                audio.start()

                loop_iter = loop_iter + 1

            if "audio" in locals():
                audio_alive = audio.is_alive()

            if (highest_priority_obj[0] is not None and "audio" in locals() and audio_alive == False):
                object_x_average = ((xmin + xmin)/2)/width
                direction = (object_x_average * calibData.getFov(dai.CameraBoardSocket.RGB)) - (calibData.getFov(dai.CameraBoardSocket.RGB)/2)
                if direction < -15:
                    body_rel_direction = "left"
                elif direction > 15:
                    body_rel_direction = "right"
                else:
                    body_rel_direction = "center"
                audio = "text_to_speech.mp3"
                speech.direction_distance(audio, round((detection.spatialCoordinates.z/1000),2), abs(round(direction)), highest_priority_obj[0], body_rel_direction)

                #speech.speed_setter(audio, 2)

                audio = multiprocessing.Process(target=speech.play_audio, args=((audio),))
                audio.start()

            cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
            cv2.imshow("depth", depthFrameColor)
            cv2.imshow("rgb", frame)

            if cv2.waitKey(1) == ord('q'):
                break

if __name__ == "__main__":
    main()
