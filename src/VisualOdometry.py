import cv2
import cv2.aruco as aruco
import numpy as np


class VisualOdometry:

    def __init__(self, video_url: str, marker_type: int, show_video: bool = True, camera_matrix=np.array([[260.0, 0.0, 640.0], [0.0, 260.0, 480.0], [0.0, 0.0, 1.0]], dtype=np.float32), dist_coeff=np.zeros(5)):
        self.video_url = video_url
        self.marker_type = marker_type
        self.show_video = show_video
        self.camera_matrix = camera_matrix
        self.dist_coeff = dist_coeff

        self.cap = cv2.VideoCapture(self.video_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        if not self.cap.isOpened():
            raise Exception(f"Error: Unable to connect to stream at {self.video_url}")


    def get_frame(self):
        _, frame = self.cap.read()
        return frame

    def process_frame(self, frame):
        if frame is None:
            return None, None
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        if self.show_video:
            cv2.imshow('Gazebo Aruco Detection', frame)
        
        return ids, corners

    def get_position(self, frame, corners, ids):
        if ids is None or corners is None or frame is None:
            return None

        marker_length = 1.0 # Marker size in meters
        obj_points = np.array([
            [-marker_length/2,  marker_length/2, 0],
            [ marker_length/2,  marker_length/2, 0],
            [ marker_length/2, -marker_length/2, 0],
            [-marker_length/2, -marker_length/2, 0]
        ], dtype=np.float32)

        for i in range(len(ids)):
            img_points = corners[i][0]
            success, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeff)

            if success:
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeff, rvec, tvec, 0.05)

                R, _ = cv2.Rodrigues(rvec)
                camera_world_pos = -R.T @ tvec

                cv2.putText(frame, f"X: {camera_world_pos[0][0]:.2f} m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                cv2.putText(frame, f"Y: {camera_world_pos[1][0]:.2f} m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                cv2.putText(frame, f"Z: {camera_world_pos[2][0]:.2f} m", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                
                if self.show_video:
                    cv2.imshow('Gazebo Aruco Detection', frame)

                return camera_world_pos.flatten()



if __name__ == "__main__":
    video_url = "udp://127.0.0.1:5001?fifo_size=0&overrun_nonfatal=1"
    marker_type = aruco.DICT_4X4_50
    vo = VisualOdometry(video_url, marker_type)

    while True:
        frame = vo.get_frame()
        ids, corners = vo.process_frame(frame)
        coordinates = vo.get_position(frame, corners, ids)
        print(f"Estimated Position: X={coordinates[0]:.2f} m, Y={coordinates[1]:.2f} m, Z={coordinates[2]:.2f} m")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break