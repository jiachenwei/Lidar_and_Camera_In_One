/**
 * @file camera_proccess.hpp
 * @brief 摄像头处理程序，主要包括：1)摄像头图像的去畸变，2)摄像头图像的目标检测
 * @author Chenwei Jia (cwjia98@gmail.com)
 * @version 1.0
 * @date 2021-05-31
 */

#include <core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "../include/darknet/darknet.h"
