import sys
from ultralytics import YOLO

# Trains yolov11 models using the existing implementation from ultralytics, trains the model
#   from using yolo11n.pt as a starting point.
#
# Usage: python3 modelTrainer.py <dataset_name>
#

# Check args
if len(sys.argv) == 2:
    dataset_name = sys.argv[1]    
else:
    print("Usage: python3 modelTrainer.py <dataset_name>")
    sys.exit(-1)


model = YOLO("models/yolo11n.pt")

model.train(
    data=("datasets/" + dataset_name + "/data.yaml"),
    epochs=100,
    save=True,
    save_period=5,
)