import cv2

class VisualOdometry:

    def __init__(self, video_url: str):
        self.video_url = video_url

        self.cap = cv2.VideoCapture(self.video_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise Exception(f"Error: Unable to connect to stream at {self.video_url}")



if __name__ == "__main__":
    video_url = "udp://127.0.0.1:5001?fifo_size=0&overrun_nonfatal=1"
    vo = VisualOdometry(video_url)

    while True:
        ret, frame = vo.cap.read()
        cv2.imshow('Gazebo Aruco Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break