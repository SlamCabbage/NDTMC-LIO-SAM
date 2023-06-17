import os
import subprocess
import time

def launch_rosbag_playback(launch_file, bag_file):
    # 启动launch文件
    launch_cmd = ['roslaunch', launch_file]
    launch_process = subprocess.Popen(launch_cmd)

    # 等待一段时间以确保ROS节点已启动
    time.sleep(5)

    # 播放bag文件
    play_cmd = ['rosbag', 'play', bag_file]
    subprocess.call(play_cmd)

    # 关闭launch进程
    launch_process.terminate()
    launch_process.wait()

if __name__ == '__main__':
    # 设置launch文件和bag文件的路径
    launch_file = '/home/liaolizhou/liosam/Myproject/src/SC-LIO-SAM/launch/run.launch'
    bag_files = [
        '/media/liaolizhou/Dataset1/KITTI/kitti_2011_09_30_drive_0027_synced.bag'
    ]

    for bag_file in bag_files:
        launch_rosbag_playback(launch_file, bag_file)
