from flask import Flask, render_template# Flask
import json


from ament_index_python.packages import get_package_share_directory

# ROS imports to setup the node
import rclpy



from plc_msgs.msg import CarierData

# Imports for threading operations
import sys
from threading import Thread
import atexit

plc_stations = [{"Stationid":-1,"last_carrierid":-1, "read_time":"none"} for i in range(16)]


##### Setting up the ROS node:
def callback(msg):
    global new_image
    node.get_logger().info('I heard: "%s"' % msg)
    plc_stations[msg.stationid-1] = {"Stationid":msg.stationid,"last_carrierid":msg.carrierid, "read_time":msg.readtime}



# Initializing the node
rclpy.init(args=None)
node = rclpy.create_node('web_gui')

# start the ROS node called Show_image_python in a new thread
Thread(target=lambda:node).start() # Starting the Thread with a target in the node
Thread(target=lambda:rclpy.spin(node)).start() # Starting the Thread with a target in the node
subscription = node.create_subscription(CarierData,'/plc_out', callback, 10) # Subscriber to the /image_name topic

# create flask app
app = Flask(__name__,root_path=get_package_share_directory("plcbind"))

# spin ROS once and refresh the node
def get_image():
    return plc_stations

# main flask page gets the image and renders
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def data():
    plc_stations = json.dumps(get_image())
    return plc_stations


#defining function to run on shutdown
def close_running_threads():
    rclpy.shutdown()
    print("closed ROS")
    sys.exit(0)


## Main funcion, only initiate the Flask app
def main(args=None):
    atexit.register(close_running_threads) # call the function to close things properly when the server is down
    app.config['SEND_FILE_MAX_AGE_DEFAULT'] = -1
    app.run(host='0.0.0.0', port=5000 ,debug=False)




if __name__ == '__main__':
    main()

