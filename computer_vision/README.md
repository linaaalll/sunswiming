# Computer Vision Pipeline — README

Quick overview of how object detection goes from the ZED camera to a usable position for navigation, and what each file does.


# Importent
make sure these are set to run custom model
- in `/home/jetson/sUNSWim/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml` set detection_model under object_detection to `CUSTOM_YOLOLIKE_BOX_OBJECTS` if you want to run te custom model else set it to `MULTI_CLASS_BOX_FAST` for the default zed cam model


## start up commands
```
ZED camera startup command with and without custom model
  -> ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm custom_object_detection_config_path:='/home/jetson/sUNSWim/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/config/custom_object_detection.yaml'  (with custom model)

  -> ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm   (without custom model)
```

## Pipeline overview

```
ZED camera
  -> zed-ros2-wrapper (runs YOLO model via TensorRT, does 3D fusion)
  -> zed_bridge_node.py (color classification + world-frame transform)
  -> /detection topic (ObjectDetection.msg)
```

The ZED wrapper does object detection itself does not run YOLO separately. We give it our trained `.onnx` model, and it publishes 3D-fused detections on `/zed/zed_node/obj_det/objects`. bridge node picks those up, works out the color for flares/buckets, converts position into map coordinates, and republishes in the format the rest of the stack expects.

## Files

### `custom_object_detection.yaml`
Config file for the ZED wrapper's object detection module. Lists our trained classes (`gate`, `flag`, `flare`, `cbuckets`, `tubs`), maps each to the class ID the model was trained with, and sets per-class filtering (confidence threshold, expected size, tracking behaviour). Points at the `.onnx` model file via `custom_onnx_file`. This is what actually turns the wrapper's detection from the default COCO model into the passed in at launch via `custom_object_detection_config_path`.

### `zed_bridge_node.py`
The main ROS2 node. Subscribes to the ZED wrapper's detection output and the camera's raw image (synced by timestamp), then for each detection:
- crops the image to the detection's bounding box
- calls `color_detection.py` to work out the color, if the class needs one
- transforms the position from camera frame into `map` frame via TF2
- publishes the result on `/detection` as an `ObjectDetection` message

This is what `Maplib` subscribes to.

### `color_detection.py`
Helper module, imported by `zed_bridge_node.py`. Does color classification on a cropped image using YUV thresholding (not HSV) — checks how much of the crop matches each candidate color and requires a clear margin over the others before deciding. Two functions: `get_flare_color` (red/yellow/blue) and `get_bucket_color` (red/blue/gray). `classify_object_color()` is the single entry point the bridge node actually calls — it picks the right function based on the detected class name. If color can't be determined confidently, returns `None` and the object still gets published, just without a color prefix on its name.

### `test_model.py`
Standalone script, not part of the ROS pipeline. Runs the raw `.pt` model against a folder of images and prints/saves what it detects (class, confidence, box). Used to check the model itself is working, independent of the ZED SDK or ROS, by testing on regular image files (e.g. underwater test photos vs dry-room test photos).

## Key things to know

- **Model runs on-device via TensorRT**, not through Python/Ultralytics, when deployed the `.onnx` export and `custom_object_detection.yaml` are what control this, not any Python inference code.
- **Class names must match exactly** between training, `custom_object_detection.yaml`, and `color_detection.py`'s `FLARE_LIKE_CLASSES`/`BUCKET_LIKE_CLASSES` sets. A mismatch fails silently — it just labels things wrong, it doesn't error.
- **Color classification only runs on classes listed in `FLARE_LIKE_CLASSES` / `BUCKET_LIKE_CLASSES`** inside `color_detection.py`. Everything else passes through with its plain class name.
- **Underwater color cast affects everything color-related.** Thresholds in `color_detection.py` need to recalibrate against real pool footage before relying on the.

