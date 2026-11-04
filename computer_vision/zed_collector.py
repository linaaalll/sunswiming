# Zed camera class performs all functionality for zed camera


import cv2
import pyzed.sl as sl

import time


# imported the whole class cos it was already there
class FrontCam:
    def __init__(self) -> None:
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = 30

        # object detection parameters
        # detection_parameters = sl.ObjectDetectionParameters()
        # detection_parameters.detection_model = sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST
        # detection_parameters.image_sync = True
        # detection_parameters.enable_tracking = True

        # position tracking allows the camera to track its own position
        # self.zed.enable_positional_tracking()

        # load object detection model used
        # err = self.zed.enable_object_detection(detection_parameters)
        # if err != sl.ERROR_CODE.SUCCESS:
        #     print(f"Object Detection: {err}, exit program.")
        #     exit(1)

        # open the cam
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Camera Open : {err}. Exit program.")
            exit(1)

        # setup runtime parameters for the zed camera
        self.runtime_parameters = sl.RuntimeParameters()
        # self.detection_runtime_parameters = sl.ObjectDetectionRuntimeParameters()
        # self.detection_runtime_parameters.detection_confidence_threshold = 40

        # self.objects = sl.Objects()


    def __del__(self) -> None:
        self.zed.disable_object_detection()
        self.zed.close()


    def _getImage(self, view):
        image = sl.Mat()
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image, view)
            return image.get_data()


    def getLeftImage(self):
        """
            Gets image from left camera,
            if no image could be retrieved returns None.
        """
        return self._getImage(sl.VIEW.LEFT)

    
    def getRightImage(self):
        """
            Gets image from right camera,
            if no image could be retrieved returns None.
        """
        return self._getImage(sl.VIEW.RIGHT)
    

    def detectObjects(self):
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            err = zed.retrieve_objects(self.objects, self.detection_runtime_parameters)
            if self.objects.is_new:
                print(f"{len(self.objects.object_list)} Object(s) detected")
                # display objects
                for obj in self.objects.object_list:
                    pos = obj.position
                    print(f"3D position : [{pos[0]},{pos[1]},{pos[2]}]")
                    bbox = obj.bounding_box
                    print(" Bounding box 3D : ")
                    for it in bbox:
                        print(f" {it}", end='')
    

import os


# testing
if __name__ == "__main__":
    # key = ''
    # window_name = "zed_test"
    # cv2.namedWindow(window_name)
    # while key != 113:
    #     image = zed.getRightImage()
    #     if image:
    #         cv2.imshow(window_name, image)
    #         key = cv2.waitKey(1)
    # cv2.destroyAllWindows()

    directory =  "images/"
    os.chdir(directory)

    zed = FrontCam()
    count = 0
    while True:
        # check this actually works lol (it should)
        leftImage = zed.getLeftImage()
        rightImage = zed.getRightImage()
        if leftImage is not None:
            cv2.imwrite("zedImage" + str(count)+ ".jpg", leftImage)
            count += 1
        if rightImage is not None:
            cv2.imwrite("zedImage" + str(count) + ".jpg", rightImage)
            count += 1
        time.sleep(0.5)


        
