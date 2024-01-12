import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, render_template, Response

app = Flask(__name__)
node = None
bridge = CvBridge()
frame = None

class ImageSubscriberNode(Node):
    def __init__(self):
        super().__init__('image_subscriber_node')
        qos_profile = rclpy.qos.qos_profile_sensor_data
        
        self.subscription = self.create_subscription(
            Image,
            'image_topic',  # Replace with the actual image topic
            self.image_callback,
            qos_profile  # Use the QoS profile
        )

    def image_callback(self, msg):
        global frame
        frame = bridge.imgmsg_to_cv2(msg)

def generate_frames():
    while True:
        if frame is not None:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_encoded = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_encoded + b'\r\n')


@app.route('/')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def ros_thread():
    global node
    rclpy.init()
    node = ImageSubscriberNode()
    rclpy.spin(node)

if __name__ == '__main__':
    import threading

    ros_thread_instance = threading.Thread(target=ros_thread, daemon=True)
    ros_thread_instance.start()

    app.run(host='0.0.0.0', port=5000,debug=True, threaded=True, use_reloader=False)
