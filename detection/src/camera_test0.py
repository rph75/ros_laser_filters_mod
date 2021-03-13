import cv2

def gstreamer_pipeline_1(
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
    ).format(sensor_id, display_width, display_height, framerate, flip_method)

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


def gstreamer_pipeline_3(
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
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink"
            % (
                sensor_id,
                sensor_mode,
                capture_width,
                capture_height,
                framerate,
                flip_method,
            )
    )

def gstreamer_pipeline_4(
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
            "video/x-raw(memory:NVMM), format=(string)NV12 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id,
                sensor_mode,
                flip_method,
                display_width,
                display_height,
            )
    )

def show_size(img,pipe):
    height, width, channels = img.shape
    text= "Pipe {}: {} x {}, {} channels".format(pipe,width,height,channels)
    cv2.putText(img, text,
                (40,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0,0,0),
                2)
    return img

cam_width = 3280
cam_height = 2464
scaling = 0.2
#pipe0=gstreamer_pipeline_2(sensor_id=1,sensor_mode=3,capture_width=800,capture_height=600,display_width=800,display_height=600,framerate=30,flip_method=2)
#pipe1=gstreamer_pipeline_2(sensor_id=0,sensor_mode=3,capture_width=800,capture_height=600,display_width=800,display_height=600,framerate=30,flip_method=2)
#pipe0=gstreamer_pipeline_4(sensor_id=0,sensor_mode=3,capture_width=800,capture_height=600,display_width=800,display_height=600,framerate=30,flip_method=2)
pipe=gstreamer_pipeline_4(sensor_id=0,sensor_mode=3,capture_width=800,capture_height=600,display_width=800,display_height=600,framerate=30,flip_method=2)
#cap1 = cv2.VideoCapture(pipe1,cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture(pipe,cv2.CAP_GSTREAMER)

while True:
    ret, image = cap.read()
    image = show_size(image,0)
    image = cv2.resize(image, (0, 0), fx=0.3, fy=0.3)
    cv2.imshow('Cam test', image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        cap.release()
        break


