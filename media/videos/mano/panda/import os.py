import os
from moviepy.editor import VideoFileClip


def time(path):
    """
    :param path: 文件路径
    :return: 返回视频文件时长（秒）
    """
    clip = VideoFileClip(path)
    return clip.duration


def file_path():
    """
    获取当前目录路径
    """
    path = os.path.dirname(os.path.abspath(__file__))
    return path


def find_mp4(path):
    """
    获取当前路径下的所有MP4格式的文件（理论上任意视频格式都可以）
    :param path: 文件路径
    :return: MP4文件列表
    """
    file_mp4 = os.listdir(path)
    mp4_list = list()
    for file in file_mp4:
        if file.rsplit(".", 1)[-1] == "mp4":
            mp4_list.append(file)
    return mp4_list


if __name__ == "__main__":
    mp4_file_list = find_mp4(file_path())
    mp4_file_path_list = list()
    time_list = list()
    for mp4_file in mp4_file_list:
        time_data = time(os.path.join(file_path(), mp4_file))
        time_list.append(float(time_data))
        print("文件名:{}，时长:{}秒".format(mp4_file, time_data))
    print("总时长{}秒".format(sum(time_list)))