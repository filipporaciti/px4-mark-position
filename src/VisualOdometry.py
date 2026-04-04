import cv2
import cv2.aruco as aruco


class VisualOdometry:

    def __init__(self, video_url: str, marker_type: int, show_video: bool = True):
        self.video_url = video_url
        self.marker_type = marker_type
        self.show_video = show_video

        self.cap = cv2.VideoCapture(self.video_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        if not self.cap.isOpened():
            raise Exception(f"Error: Unable to connect to stream at {self.video_url}")


    def process_frame(self):
        _, frame = self.cap.read()
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        if self.show_video:
            cv2.imshow('Gazebo Aruco Detection', frame)


if __name__ == "__main__":
    video_url = "udp://127.0.0.1:5001?fifo_size=0&overrun_nonfatal=1"
    marker_type = aruco.DICT_4X4_50
    vo = VisualOdometry(video_url, marker_type)

    while True:
        vo.process_frame()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break