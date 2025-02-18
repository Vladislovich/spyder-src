#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import os
import sys

# Получаем путь к текущей директории
current_dir = os.path.dirname(os.path.realpath(__file__))
log_file_path = os.path.join(current_dir+"/logs", "pointcloud_log_3.txt")

def cloud_callback(cloud):
    # Преобразуем PointCloud2 в список точек
    points = list(pc2.read_points(cloud, field_names=("x", "y", "z", "intensity", "ring", "time"), skip_nans=True))

    log_message = "-----------Received PointCloud with %d points-----------\n" % len(points)
    rospy.loginfo(log_message.strip())

    # Открываем файл для записи
    with open(log_file_path, "a") as log_file: #a
        i = 1
        log_file.write(log_message)
        log_file.write(f"x      y       z    intens\n")
        for point in points:
            x, y, z, intensity, ring, time = point            
            rospy.loginfo(f"{i}: {x:.4f} {y:.4f} {z:.4f} {intensity:.0f} {ring:.0f}\n")
            log_file.write(f"{x:.4f} {y:.4f} {z:.4f} {intensity:.0f}\n")
            i+=1
    rospy.loginfo("Данные записаны в файл output.txt")
    sys.exit("exit")

def main():
    rospy.init_node('pointcloud2_to_file', anonymous=True)
    rospy.Subscriber("/velodyne_points", PointCloud2, cloud_callback)
    rospy.spin()

if __name__ == '__main__':
    main()