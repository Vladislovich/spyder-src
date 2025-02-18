#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import os
import sys

# Получаем путь к текущей директории
current_dir = os.path.dirname(os.path.realpath(__file__))
log_file_path = os.path.join(current_dir, "logs", "lidar_scan_log.txt")

def laser_scan_callback(scan):
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
            z = 0  # Лидар 2D, поэтому z = 0
            points.append((x, y, z, inten))

    log_message = f"-----------Received LaserScan with {len(points)} points-----------\n"
    rospy.loginfo(log_message.strip())

    # Открываем файл для записи
    os.makedirs(os.path.dirname(log_file_path), exist_ok=True)  # Создаем директорию, если она не существует
    with open(log_file_path, "a") as log_file:
        log_file.write(log_message)
        log_file.write(f"x      y       z\n")
        for i, point in enumerate(points, start=1):
            x, y, z, inten = point
            log_file.write(f"{x:.6f} {y:.6f} {z:.6f} {inten:.0f}\n")
    
    rospy.loginfo("Данные записаны в файл lidar_scan_log.txt")
    sys.exit("exit")  # Завершаем выполнение

def main():
    rospy.init_node('lidar_scan_to_file', anonymous=True)
    rospy.Subscriber("/MS200/scan", LaserScan, laser_scan_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
