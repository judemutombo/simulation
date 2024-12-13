#!/usr/bin/env python3

import flask
import socketio
import rospy
from std_msgs.msg import String, Bool, Byte, Int32
from ie_communication.msg import DirectionEnum, SensorDataMap
from flask_cors import CORS
from ie_communication.srv import camState, camStateResponse


class ie_API_Server:
    def __init__(self):
        self.app = flask.Flask(__name__)
        
        # Initialize CORS with wildcard to allow any origin
        CORS(self.app, resources={r"/*": {"origins": "*"}})

        # Manually add CORS headers after each request
        @self.app.after_request
        def apply_cors(response):
            response.headers["Access-Control-Allow-Origin"] = "*"
            response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
            response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization"
            return response

        # Initialize SocketIO with CORS allowed
        self.sio = socketio.Server(cors_allowed_origins="*")
        self.app.wsgi_app = socketio.WSGIApp(self.sio, self.app.wsgi_app)

        # Define publishers and subscribers
        self.mc_pub = rospy.Publisher("motor_controller", Int32, queue_size=10)
        #self.cam_pub = rospy.Publisher("camera_state", Bool, queue_size=10)
        
        self.cam_sub = rospy.Subscriber("camera_feed", Byte, self.cameraFeedCallback)
        self.sensors_sub = rospy.Subscriber("sensor_data", SensorDataMap, self.sensorsCallback)

        # Register event handlers
        self.sio.on("connect", self.onConnect)
        self.sio.on("disconnect", self.onDisconnect)
        self.sio.on("cameraState", self.cameraStateChange)
        self.sio.on("moveDirection", self.movement)
        self.sio.on("message", self.message)

    def sensorsCallback(self, data):
        pass

    def cameraFeedCallback(self,data):
        print(data)
        

    def onConnect(self, sid, environ):
        print(f"client {sid} connected")
    
    def onDisconnect(self, sid):
        print(f"client {sid} disconnected")

    def cameraStateChange(self, sid, state):
        print(state)
        rospy.wait_for_service('camera_state')
        camera_state = rospy.ServiceProxy('camera_state', camState)

        try:
            if state["state"]:
                camera_state(True)
            else:
                camera_state(False)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


    def movement(self, sid, direction):
        print(direction["direction"])
        if direction["direction"] == "UP" :
            self.mc_pub.publish(1)
        elif direction["direction"] == "DOWN" :
            self.mc_pub.publish(2)
        elif direction["direction"] == "LEFT" :
            self.mc_pub.publish(3)
        elif direction["direction"] == "RIGHT":
            self.mc_pub.publish(4)
        elif direction["direction"] == "STOP":
            self.mc_pub.publish(5)

    def connect(self):
        self.app.run(host="0.0.0.0", port=5000)

    def message(self, sid, message):
        print(f"client {sid} : {message}")

if __name__ == '__main__':
    from threading import Thread
    
    Thread(target=lambda: rospy.init_node('ie_Api', disable_signals=True)).start()
    server = ie_API_Server()
    server.connect()
    rospy.spin()
