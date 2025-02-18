#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import os
import sys
import time

#roslaunch velodyne_pointcloud VLP16_points.launch
#rosrun rviz rviz -f velodyne

last_processed_time = 0
process_frequency = 1.0

# Получаем путь к текущей директории
current_dir = os.path.dirname(os.path.realpath(__file__))
#log_file_path = os.path.join(current_dir, "..//..//exper_data", "vld_n45_50_1.txt")

activate_flag = False
arduino_angle = 0

def velodyne_scan_callback(cloud):
    global last_processed_time
    if time.time() - last_processed_time >= 1.0 / process_frequency:
        last_processed_time = time.time()
        global activate_flag
        ard_ang = arduino_angle
        if activate_flag == False:
            rospy.loginfo("Deactive_flag")
            return
        activate_flag = False
        # Преобразуем PointCloud2 в список точек
        points = list(pc2.read_points(cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True))

        log_message = f"-----------Received Velodyne with {len(points)} points-----------\n"
        rospy.loginfo(log_message.strip())

        # Открываем файл для записи
        filename = input("Введите файл: ")
        filename = "vld_pov45_" + filename + ".txt"
        log_file_path = os.path.join(current_dir, "..//..//exper_data", filename)
        os.makedirs(os.path.dirname(log_file_path), exist_ok=True)
        with open(log_file_path, "a") as log_file: #a
            #log_file.write(log_message)
            #log_file.write(f"x      y       z    intens\n")
            for point in points:
                x, y, z, intensity = point            
                log_file.write(f"{ard_ang:.4f} {x:.4f} {y:.4f} {z:.4f} {intensity:.0f}\n")

        rospy.loginfo("Данные записаны в файл:")
        rospy.loginfo(log_file_path)
        sys.exit("exit")

def flag_callback(msg):
    global activate_flag
    if msg.data == 1:
        #rospy.loginfo("Activate flag")
        activate_flag = True
    else:
        #rospy.loginfo("Deactivate flag")
        activate_flag = False

def arduino_callback(msg):
    global arduino_angle
    if msg.data != 0:
        #rospy.loginfo("Angle is received")
        arduino_angle = msg.data
    else:
        rospy.loginfo("Angle not received")

def main():
    rospy.init_node('velodyne_sub', anonymous=True)
    rospy.Subscriber("/activate_flag", Int32, flag_callback)
    rospy.Subscriber("/velodyne_points", PointCloud2, velodyne_scan_callback)
    rospy.Subscriber("/arduino_angle", Float32, arduino_callback)
    rospy.spin()

if __name__ == '__main__':
    main()