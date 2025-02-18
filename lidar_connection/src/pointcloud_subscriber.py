#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import os


# Получаем путь к текущей директории
current_dir = os.path.dirname(os.path.realpath(__file__))
log_file_path = os.path.join(current_dir, 'pointcloud_log.txt')

def pointCloudCallback(cloud_msg):
    # Вывод количества точек в облаке
    points = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))
    log_message = "-----------Received PointCloud with %d points-----------\n" % len(points)
    rospy.loginfo(log_message.strip())

    # Запись логов в файл
    with open(log_file_path, 'a') as log_file:
        log_file.write(log_message)

        # Запись координат каждой точки
        for point in points:
            x, y, z = point
            point_message = "x=%f, y=%f, z=%f\n" % (x, y, z)
            rospy.loginfo(point_message.strip())
            log_file.write(point_message)

def main():
    rospy.init_node('pointcloud_subscriber', anonymous=True)

    # Подписка на топик с PointCloud2
    rospy.Subscriber("/velodyne_points", PointCloud2, pointCloudCallback)

    rospy.spin()

if __name__ == '__main__':
    main()