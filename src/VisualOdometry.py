import cv2
import cv2.aruco as aruco
import numpy as np
import math
import json


class VisualOdometry:

    def __init__(self, video_url: str, marker_type: int, camera_matrix: np.ndarray, show_video: bool = True, dist_coeff=np.zeros(5), marker_info_path="src/marker_info/aruco_floor_sim.json"):
        self.video_url = video_url
        self.marker_type = marker_type
        self.show_video = show_video
        self.camera_matrix = camera_matrix
        self.dist_coeff = dist_coeff
        self.marker_info = json.load(open(marker_info_path, "r"))

        self.obj_points = np.array([
            [-self.marker_info["marker_length"]/2,  self.marker_info["marker_length"]/2, 0],
            [ self.marker_info["marker_length"]/2,  self.marker_info["marker_length"]/2, 0],
            [ self.marker_info["marker_length"]/2, -self.marker_info["marker_length"]/2, 0],
            [-self.marker_info["marker_length"]/2, -self.marker_info["marker_length"]/2, 0]
        ], dtype=np.float32)

        self.cap = cv2.VideoCapture(self.video_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        if not self.cap.isOpened():
            raise Exception(f"Error: Unable to connect to stream at {self.video_url}")

    def get_covariance_matrix(self, coordinates, l):
        dev_xy = coordinates[2]/self.camera_matrix[0, 0]
        dev_z = coordinates[2]/l
        dev_yaw = math.radians(0.1)

        v_x = dev_xy**2
        v_y = dev_xy**2
        v_z = dev_z**2
        v_roll = dev_yaw**2
        v_pitch = dev_yaw**2
        v_yaw = dev_yaw**2

        cov_matrix = [
            v_x, 0, 0, 0, 0, 0,
                v_y, 0, 0, 0, 0,
                    v_z, 0, 0, 0,
                        v_roll, 0, 0,
                            v_pitch, 0,
                                v_yaw
        ]
        return cov_matrix

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
    
    def print_text(self, frame, position, yaw):
        cv2.putText(frame, f"X: {position[0][0]:.2f} m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        cv2.putText(frame, f"Y: {position[1][0]:.2f} m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        cv2.putText(frame, f"Z: {position[2][0]:.2f} m", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        cv2.putText(frame, f"Yaw: {yaw:.1f} rad", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        if self.show_video:
            cv2.imshow('Gazebo Aruco Detection', frame)

    def get_l(self, img_points):
        c = img_points.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = c
        side1 = np.linalg.norm(top_left - top_right)
        side2 = np.linalg.norm(top_right - bottom_right)
        side3 = np.linalg.norm(bottom_right - bottom_left)
        side4 = np.linalg.norm(bottom_left - top_left)
        l = (side1 + side2 + side3 + side4) / 4
        return l

    def get_position(self, frame, corners, ids):
        if ids is None or corners is None or frame is None:
            return None, None, self.get_covariance_matrix([0, 0, 0], 1)

        for i in range(len(ids)):

            img_points = corners[i][0]
            success, rvec, tvec = cv2.solvePnP(self.obj_points, img_points, self.camera_matrix, self.dist_coeff)

            if success:
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeff, rvec, tvec, 0.05)

                R, _ = cv2.Rodrigues(rvec)
                camera_world_pos = -R.T @ tvec

                if str(ids[i][0]) not in self.marker_info["position"]:
                    print(f"Warning: Marker ID {ids[i][0]} not found in marker_info. Skipping position adjustment.")
                    continue

                camera_world_pos[0] = self.marker_info["position"][str(ids[i][0])]["x"] + camera_world_pos[0]
                camera_world_pos[1] = self.marker_info["position"][str(ids[i][0])]["y"] + camera_world_pos[1]

                camera_world_pos = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]]) @ camera_world_pos

                yaw = -math.atan2(R[1, 0], R[0, 0])

                self.print_text(frame, camera_world_pos, yaw)

                camera_world_pos = camera_world_pos.flatten()

                l = self.get_l(corners[i][0])

                return camera_world_pos, yaw, self.get_covariance_matrix(camera_world_pos, l)

        return None, None, self.get_covariance_matrix([0, 0, 0], 1)

if __name__ == "__main__":
    video_url = "udp://127.0.0.1:5001?fifo_size=0&overrun_nonfatal=1"
    marker_type = aruco.DICT_4X4_50
    camera_matrix = np.array([
        [537.0, 0.0, 640.0], 
        [0.0, 537.0, 480.0], 
        [0.0, 0.0, 1.0]], 
        dtype=np.float32)
    
    vo = VisualOdometry(video_url, marker_type, camera_matrix)

    while True:
        frame = vo.get_frame()
        ids, corners = vo.process_frame(frame)
        coordinates, yaw = vo.get_position(frame, corners, ids)
        if coordinates is not None and yaw is not None:
            print(f"Estimated Position: X={coordinates[0]:.2f} m, Y={coordinates[1]:.2f} m, Z={coordinates[2]:.2f} m", f"Yaw={yaw:.1f} rad")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break