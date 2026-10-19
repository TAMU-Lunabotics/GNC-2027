# All tags detected in one camera image. One message per image, even if empty
std_msgs/Header header                 # frame_id = camera optical frame, stamp = image capture time
TagDetection[] detections
