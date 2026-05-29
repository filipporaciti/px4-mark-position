import cv2
import numpy as np

class TelemetryVisualizer:
    def __init__(self, width=1500, height=400, show_video: bool = True):
        self.width = width
        self.height = height
        self.plot_h = height // 3
        self.max_points = width - 60
        
        self.data = {'X': [], 'Y': [], 'Z': []}

        self.colors = {'X': (0, 0, 255), 'Y': (0, 255, 0), 'Z': (255, 100, 0)}
        self.target_color = (0, 255, 255) 

        self.show_video = show_video
        self._render()

    def update(self, val_x, val_y, val_z, target_x=0, target_y=0, target_z=0):
        if not self.show_video:
            return
        
        targets = [target_x, target_y, target_z]
        values = [val_x, val_y, val_z]
        
        for i, key in enumerate(['X', 'Y', 'Z']):
            self.data[key].append((values[i], targets[i]))
            if len(self.data[key]) > self.max_points:
                self.data[key].pop(0)
        
        self._render()

    def _render(self):
        if not self.show_video:
            return
        
        canvas = np.ones((self.height, self.width, 3), dtype=np.uint8) * 20
        
        for i, key in enumerate(['X', 'Y', 'Z']):
            y_offset = i * self.plot_h
            current_samples = self.data[key]
            
            if not current_samples: continue

            vals_only = [s[0] for s in current_samples]
            targets_only = [s[1] for s in current_samples]
            
            max_val = max(max(np.abs(vals_only)), max(np.abs(targets_only)), 1.0)
            scale = (self.plot_h / 2.5) / max_val
            zero_line = int(y_offset + (self.plot_h / 2))

            cv2.putText(canvas, f"{key} | Val: {vals_only[-1]:.1f} | Tar: {targets_only[-1]:.1f}", 
                        (10, y_offset + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.line(canvas, (0, zero_line), (self.width, zero_line), (40, 40, 40), 1)

            if len(current_samples) > 1:
                pts_val = []
                pts_target = []
                
                for x_idx, (val, tar) in enumerate(current_samples):
                    x_pos = x_idx + 40

                    y_val = int(zero_line - (val * scale))
                    y_tar = int(zero_line - (tar * scale))
                    
                    pts_val.append([x_pos, y_val])
                    pts_target.append([x_pos, y_tar])
                
                cv2.polylines(canvas, [np.array(pts_target, np.int32)], False, self.target_color, 1, cv2.LINE_AA)
                cv2.polylines(canvas, [np.array(pts_val, np.int32)], False, self.colors[key], 1, cv2.LINE_AA)

        cv2.imshow("Telemetry", canvas)
