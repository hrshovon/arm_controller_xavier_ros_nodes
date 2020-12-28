import serial
import threading
import time 
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
srlobj = serial.Serial("/dev/ttyACM0",115200,timeout=10)
time.sleep(4)
def convert_angles(data,angle_max_limits = np.array([240,90,90,180]), conversion_arr = np.array([[1004,2],
                                                                                               [474,93],
                                                                                               [987,600],
                                                                                               [2626,189]])):
    lst_angles=[]
    for raw_data,a_max,conversion_factor in zip(data,angle_max_limits,conversion_arr):
        a_min = 0
        c_min = conversion_factor[0]
        c_max = conversion_factor[1]

        angle_out = (((a_max-a_min)/(c_max-c_min))*(raw_data-c_min))+a_min
        lst_angles.append(angle_out)
    return np.array(lst_angles,dtype=np.float32)

def read_from_port(ser,publisher):
    while True:
        try:
            data = ser.readline().decode()
            if len(data)>5:
                try:
                    data_out = np.array([float(d) for d in data.replace("\n","").split(",")],dtype=np.float32)
                    msg = Float32MultiArray(data=convert_angles(data=data_out))
                    publisher.publish(msg)
                except:
                    pass
        except: 
            pass


def listener_callback(msg):
    data_out = bytearray([210]+[int(m) for m in msg.data]+[135,245])
    print(data_out)
    if srlobj!=None:
        srlobj.write(data_out)

def main(args=None):
    rclpy.init(args=args)        
    node = rclpy.create_node('hand_robot_node')
    publisher = node.create_publisher(Float32MultiArray, 'robot_sensor_data')
    subscriber = node.create_subscription(
            Float32MultiArray,
            '/robot_command',
            listener_callback)

    thread = threading.Thread(target=read_from_port, args=(srlobj,publisher,))
    thread.start()
    srlobj.flushInput()
    srlobj.flushOutput()
    time.sleep(1)
    #robotdata_publisher = robot_data_publisher()
    #while rclpy.ok:
    rclpy.spin(node)    
    #rclpy.spin_once(robotdata_publisher,timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
        
    srlobj.close()

if __name__ == '__main__':
    main()