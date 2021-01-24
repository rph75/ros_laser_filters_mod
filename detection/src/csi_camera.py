import cv2
import threading
import numpy as np

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of each camera pane in the window on the screen

left_camera = None
right_camera = None


class CSI_Camera:

    def __init__(self):
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    def open(self, gstreamer_pipeline_string):
        try:
            self.video_capture = cv2.VideoCapture(
                gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )

        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)
            return
        # Grab the first frame to start the video capturing
        self.grabbed, self.frame = self.video_capture.read()

    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        return self

    def stop(self):
        self.running = False
        self.read_thread.join()

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Could not read image from camera")
        # FIX ME - stop and cleanup thread
        # Something bad happened

    def read(self):
        with self.read_lock:
            frame = None if self.frame is None else self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()


# Currently there are setting frame rate on CSI Camera on Nano through gstreamer
# Here we directly select sensor_mode 3 (1280x720, 59.9999 fps)
# (Actually, a bit confusing, as the camera mode from gstreamer does not match that description)
    #   Camera mode  = 1
    #    Output Stream W = 3264 H = 1848
    #
    #    Camera mode  = 2
    #    Output Stream W = 1920 H = 1080
    #
    #    Camera mode  = 3
    #    Output Stream W = 1640 H = 1232
    #
    #    Camera mode  = 4
    #    Output Stream W = 1280 H = 720
    #
    #    Camera mode  = 5
    #    Output Stream W = 1280 H = 720
# This is the pipeline that csi_camera example is using, but giving me different aspects in the camera than what the old pipeline gave me.
def gstreamer_pipeline_2(
        sensor_id=0,
        sensor_mode=3,
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=30,
        flip_method=0,
):
    return (
            "nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id,
                sensor_mode,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
    )

# This is the pipeline which used to work from previous setup, but maybe suboptimal?
# It appears that defining the width/height directly in the camera params is actually cropping the image
# (in fact, this means that I'd get a much wider picture if I were not doing this. But everything is already calibrated
# to these settings)
# Or maybe it is simply the sensormode which is giving me trouble?
# Play with this a but more, until root cause is identified
def gstreamer_pipeline(
        sensor_id=0,
        sensor_mode=3,
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=30,
        flip_method=0,
):
    return (
        "nvarguscamerasrc sensor_id={} ! "
        "video/x-raw(memory:NVMM), width={}, height={}, format=(string)NV12, framerate=(fraction){}/1 ! "
        "nvvidconv flip-method={} ! "
        "video/x-raw, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=True"
    ).format(sensor_id,display_width,display_height,framerate,flip_method)


class CSI_Cameras:

    def __init__(self,sensor_mode,height,width,framerate,flips):
        self.sensor_mode = sensor_mode
        self.height = height
        self.width = width
        self.framerate = framerate
        self.flips = flips


    def start_cameras(self):
        self.cameras = []
        success = True
        for i in range(len(self.flips)):
            camera=CSI_Camera()
            self.cameras.append(camera)
            camera.open(
                gstreamer_pipeline(
                    sensor_id=i,
                    sensor_mode=self.sensor_mode,
                    flip_method= self.flips[i],
                    display_height=self.height,
                    display_width=self.width,
                    framerate=self.framerate
                )
            )
            camera.start()
            success = success & camera.video_capture.isOpened()
        return success

    def read_camera(self,i):
        return self.cameras[i].read()

    def stop_cameras(self):
        for camera in self.cameras:
            camera.stop()
            camera.release()


if __name__ == "__main__":
    cameras=CSI_Cameras(sensor_mode=3,height=600,width=800,framerate=15,flips=[0,2])
    if cameras.start_cameras():
        # cv2.namedWindow("CSI Cameras", cv2.WINDOW_AUTOSIZE)
        # while cv2.getWindowProperty("CSI Cameras", 0) >= 0:
        #     _, left_image = cameras.read_camera(0)
        #     _, right_image = cameras.read_camera(1)
        #     camera_images = np.hstack((left_image, right_image))
        #     cv2.imshow("CSI Cameras", camera_images)
        #
        #     # This also acts as
        #     keyCode = cv2.waitKey(30) & 0xFF
        #     # Stop the program on the ESC key
        #     if keyCode == 27:
        #         break
        for i in range(10):
            grabbed,img0 = cameras.read_camera(0)
            grabbed,img1 = cameras.read_camera(1)
            print("{}, {}".format(str(np.shape(img0)),str(np.shape(img0))))


    cameras.stop_cameras()
    cv2.destroyAllWindows()

