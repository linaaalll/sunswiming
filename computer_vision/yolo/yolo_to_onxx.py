# converts a pretrained yolo model of the form "yolo11n.pt" to onxx format

import sys
from ultralytics import YOLO


model = YOLO(sys.argv[1])
model.export(format="onnx", task="detect")