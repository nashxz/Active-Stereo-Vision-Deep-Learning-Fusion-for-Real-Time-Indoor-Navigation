import pyrealsense2 as rs
import numpy as np
import cv2
import math
from ultralytics import YOLO
from communication import ChassisController
from ina219 import INA219
import time

# COPY AND RUN THIS ON POWERSHELL (LAPTOP) TO START THE GSTREAMER RTSP SERVER:
# cmd.exe /c '"C:\Program Files\gstreamer\1.0\msvc_x86_64\bin\gst-launch-1.0.exe" -v udpsrc port=5000 ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! rtph264depay ! decodebin ! videoconvert ! autovideosink sync=false'


# config
W, H = 640, 480
FPS = 30
D_INF = 1  
STEER_THRESHOLD = 0.4 # Lowered for faster response

current_direction = 'f' 

# hardware + stream
def setup_hardware():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    # 1. Enable Infrared stream (Index 1 is usually the left IR imager)
    config.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)
    
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    
    depth_sensor = profile.get_device().first_depth_sensor()
    if depth_sensor.supports(rs.option.emitter_enabled):
        depth_sensor.set_option(rs.option.emitter_enabled, 1)
        depth_sensor.set_option(rs.option.visual_preset, 3) 
        
    # bitrate initially 1500, now 2000
    gst_out = (
        "appsrc ! videoconvert ! video/x-raw, format=I420 ! "
        "x264enc tune=zerolatency speed-preset=ultrafast threads=4 bitrate=2000 key-int-max=30 ! "
        "rtph264pay config-interval=1 pt=96 ! "
        "udpsink host=172.20.10.4 port=5000 sync=false"
    )
    
    out = cv2.VideoWriter(gst_out, cv2.CAP_GSTREAMER, 0, FPS, (W*2, H*2), True)
    
    if not out.isOpened():
        print("GStreamer pipeline failed to open.")
        
    return pipeline, align, profile, out

# HELPER FUNCTIONS
def get_valid_depth(slice_data):
    valid = slice_data[slice_data > 0]
    return np.median(valid) if len(valid) > 0 else 0.0

# THE ARTIFICIAL POTENTIAL FIELD (APF)
def compute_apf_and_steer(depth_in_meters, yolo_results):
    global current_direction
    
    H_FOV = math.radians(87) 
    
    # ATTRACTIVE FORCE
    f_x = 0.8  
    f_y = 0.0  
    
    # REPULSIVE FORCE: RAW DEPTH
    center_row = H // 2
    strip = depth_in_meters[center_row-10:center_row+10, :]
    closest_points = np.min(strip, axis=0)
    
    buckets = np.array_split(closest_points, 20)
    eta_depth = 0.4 

    for i, bucket in enumerate(buckets):
        dist = get_valid_depth(bucket)
        if 0.05 < dist < D_INF:
            col_index = (W // 20) * i + (W // 40)
            angle = (col_index / W) * H_FOV - (H_FOV / 2)
            
            x = dist * math.cos(angle)
            y = dist * math.sin(angle) * -1 
            
            mag = eta_depth * (1.0/dist - 1.0/D_INF) / (dist**2)
            f_x -= mag * (x/dist) 
            f_y -= mag * (y/dist) * 1.2
            
    # YOLO 
    for box in yolo_results[0].boxes:
        if box.conf[0] > 0.5: 
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            obj_depth = get_valid_depth(depth_in_meters[y1:y2, x1:x2])
            
            if 0.05 < obj_depth < (D_INF + 0.5): # See objects a bit further away
                box_center_x = (x1 + x2) / 2
                angle = (box_center_x / W) * H_FOV - (H_FOV / 2)
                
                name = yolo_results[0].names[int(box.cls[0])]
                eta_yolo = 3.0 if name == 'person' else 1.5
                
                mag = eta_yolo * (1.0/obj_depth - 1.0/(D_INF+0.5)) / (obj_depth**2)
                
                # Smooth tweak: Let YOLO push sideways harder than it pushes backwards
                f_x -= mag * (math.cos(angle)) * 0.5 
                f_y -= mag * (math.sin(angle) * -1) * 1.5 
                
    # 4. DECISION MATRIX WITH HYSTERESIS
    new_direction = 'f'
    
    if f_x < 0.15: 
        if abs(f_y) > 0.3: 
            new_direction = 'l' if f_y > 0 else 'r'
        else:
            new_direction = 's'
    elif f_y > STEER_THRESHOLD:
        new_direction = 'l'
    elif f_y < -STEER_THRESHOLD:
        new_direction = 'r'
    else:
        # Hysteresis: Keep turning until the force drops significantly
        if current_direction in ['l', 'r'] and abs(f_y) > (STEER_THRESHOLD * 0.4):
            new_direction = current_direction
        else:
            new_direction = 'f'

    current_direction = new_direction
    return new_direction, f_x, f_y # Returning the forces to display on screen


# MAIN LOOP

def main():
    try:
        cv2.destroyAllWindows()
    except:
        pass
        
    pipeline, align, profile, out = setup_hardware()
    model = YOLO('/home/group26/Active-Stereo-Vision-Deep-Learning-Fusion-for-Real-Time-Indoor-Navigation/src/brain/yolov8n.engine', task='detect')
    
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    first_inference = True

    chassis = ChassisController(port="/dev/ttyTHS1", baudrate=115200)
    print("-> System Initialized. Entering autonomous loop...")

    ups_sensor = None
    try:
        ups_sensor = INA219(addr=0x41)
        print("-> UPS Sensor Initialized.")
    except Exception as e:
        print(f"-> WARNING: UPS not detected ({e}).")

    # Variables to hold the text so we don't have to read I2C every frame
    batt_str = "Battery: N/A"
    volt_str = "Voltage: N/A"
    pwr_str = "Power: N/A"
    
    last_sent_direction = None # <-- FIX: Prevents serial buffer flooding
    frame_counter = 0
    
    try:
        while True:
            frame_counter += 1
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            # 1. Grab the IR frame (unaligned, as IR and Depth share an optical center)
            ir_frame = frames.get_infrared_frame(1) 
            
            if not depth_frame or not color_frame or not ir_frame: continue

            frame = np.asanyarray(color_frame.get_data())
            depth_data = np.asanyarray(depth_frame.get_data())
            depth_in_meters = depth_data * depth_scale
            
            # 2. Process IR data (Convert 8-bit grayscale to 3-channel BGR so it stacks with color)
            ir_data = np.asanyarray(ir_frame.get_data())
            ir_image = cv2.cvtColor(ir_data, cv2.COLOR_GRAY2BGR)
            
            # 3. Process Depth data for viewing (Apply a colormap)
            depth_visual = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(depth_visual, cv2.COLORMAP_JET)

            # Poll battery every 30 frames to prevent I2C bottlenecking
            if ups_sensor and frame_counter % 30 == 0:
                try:
                    bus_voltage = ups_sensor.getBusVoltage_V()
                    power_w = ups_sensor.getPower_W()
                    
                    # Calculate percentage based on 3-cell 9V-12.6V logic
                    percent = (bus_voltage - 9) / 3.6 * 100
                    percent = max(0, min(percent, 100))
                    
                    batt_str = f"Battery: {percent:.1f} %"
                    volt_str = f"Voltage: {bus_voltage:.2f} V"
                    pwr_str  = f"Power  : {power_w:.2f} W"
                except Exception:
                    pass # Ignore occasional I2C read glitches
            
            if first_inference:
                print("-> Warming up TensorRT GPU Engine...")
            results = model(frame, verbose=False)
            if first_inference:
                print("-> Sprinting at full FPS...")
                first_inference = False
                
            frame = results[0].plot()

            # The Brain: Run Artificial Potential Field
            direction, f_x, f_y = compute_apf_and_steer(depth_in_meters, results)
            
            # Send to Wheels ONLY if the decision changed
            if direction != last_sent_direction or frame_counter % 15 == 0:
                chassis.send_command(direction)
                last_sent_direction = direction

            # Heads Up Display (HUD) for Stream (Drawn on the RGB frame)
            color_map = {'f': (0, 255, 0), 'l': (0, 255, 255), 'r': (0, 255, 255), 's': (0, 0, 255)}
            text_map = {'f': 'FORWARD', 'l': 'TURN LEFT', 'r': 'TURN RIGHT', 's': 'BRAKE'}
            
            cv2.putText(frame, f"CMD: {text_map[direction]}", (10, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_map[direction], 3)
            cv2.putText(frame, f"Fx (Fwd): {f_x:.2f} | Fy (Lat): {f_y:.2f}", (10, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # 4. Create the 2x2 Grid Layout
            # Top row: RGB on left, Depth on right
            top_row = np.hstack((frame, depth_colormap))
            
            # Bottom row: IR on left, Blank (or a custom dashboard) on right
            # Bottom row: IR on left, Blank (or a custom dashboard) on right
            blank_square = np.zeros((H, W, 3), dtype=np.uint8)
            
            # Draw telemetry on the blank square
            if ups_sensor:
                # Color code battery text (Red if low, Green if good)
                batt_color = (0, 0, 255) if "N/A" not in batt_str and float(batt_str.split()[1]) < 20 else (0, 255, 0)
                
                cv2.putText(blank_square, batt_str, (50, H//2 - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, batt_color, 2)
                cv2.putText(blank_square, volt_str, (50, H//2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(blank_square, pwr_str, (50, H//2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            else:
                cv2.putText(blank_square, "Telemetry Offline", (50, H//2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
            bottom_row = np.hstack((ir_image, blank_square))
            
            # Stack rows vertically
            combined_stream = np.vstack((top_row, bottom_row))

            if out.isOpened():
                out.write(combined_stream)

    except KeyboardInterrupt:
        print("\n-> Ctrl+C detected. Shutting down gracefully...")

    finally:
        chassis.close() 
        pipeline.stop()
        if out.isOpened():
            out.release()

if __name__ == "__main__":
    main()