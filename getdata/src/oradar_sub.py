#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import math
import os
import sys
import time

#roslaunch oradar_lidar ms200_scan.launch
#roslaunch oradar_lidar ms200_scan_view.launch (使用rviz显示) 

last_processed_time = 0
process_frequency = 1.0

log_file_path = None
#log_file_path = os.path.join(current_dir, "data", "oradar_data_1.txt")

activate_flag = False
arduino_angle = 0

def oradar_scan_callback(scan):
    global last_processed_time
    if time.time() - last_processed_time >= 1.0 / process_frequency:
        last_processed_time = time.time()
        global activate_flag
        ard_ang = arduino_angle
        if activate_flag == False:
            rospy.loginfo("Deactive_flag")
            return
        activate_flag = False
        # Преобразуем LaserScan данные в точки
        angles = [scan.angle_min + i * scan.angle_increment for i in range(len(scan.ranges))]
        points = []
        intens = scan.intensities
        for r, theta, inten in zip(scan.ranges, angles, intens):
            if math.isnan(r) or math.isnan(inten):
                continue
            if r == 0: continue 
            if not math.isinf(r):  # Игнорируем бесконечные значения
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                points.append((r, theta, ard_ang, x, y, inten))

        log_message = f"-----------Received Oradar with {len(points)} points-----------\n"
        rospy.loginfo(log_message.strip())
      
        # Открываем файл для записи
        os.makedirs(os.path.dirname(log_file_path), exist_ok=True)  # Создаем директорию, если она не существует
        with open(log_file_path, "a") as log_file:
            #log_file.write(log_message)
            #log_file.write(f"r, theta, ard_ang, x, y, inten\n")
            for i, point in enumerate(points, start=1):
                r, theta, ard_ang, x, y, inten = point
                log_file.write(f"{r:.4f} {theta:.4f} {ard_ang:.4f} {x:.4f} {y:.4f} {inten:.0f}\n")
        
        rospy.loginfo("Данные записаны в файл:")
        rospy.loginfo(log_file_path)
        sys.exit("exit")  # Завершаем выполнение
    
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
    global log_file_path 
    # Открываем файл для записи
    filename = input("Введите файл: ")
    filename = "ord_n45_250_" + filename + ".txt"    
    current_dir = os.path.dirname(os.path.realpath(__file__))
    log_file_path = os.path.join(current_dir, "..//..//exper_data", filename)

    rospy.init_node('oradar_sub', anonymous=True)
    rospy.Subscriber("/MS200/scan", LaserScan, oradar_scan_callback)
    rospy.Subscriber("/activate_flag", Int32, flag_callback)
    rospy.Subscriber("/arduino_angle", Float32, arduino_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
