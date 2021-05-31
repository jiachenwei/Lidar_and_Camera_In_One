/**
 * @file core.cpp
 * @brief
 * @author Chenwei Jia (cwjia98@gmail.com)
 * @version 1.0
 * @date 2021-05-17
 */
#include "core.hpp"

#include <cmath>
#include <iostream>

/**
 * @brief 计算多个目标之间的iou矩阵
 * @param  pixel_lidar_objects    激光雷达目标的像素坐标
 * @param  pixel_camera_objects   摄像头目标的像素坐标
 * @return Eigen::MatrixXd iou矩阵
 */
Eigen::MatrixXd compute_2d_iou_matrix(
    std::vector<PixelLidarObjectRect> *pixel_lidar_objects,
    std::vector<PixelCameraObjectRect> *pixel_camera_objects) {
    uint lid_obj_num = pixel_lidar_objects->size();
    uint cam_obj_num = pixel_camera_objects->size();
    Eigen::MatrixXd iou_mat(lid_obj_num, cam_obj_num);
    for (size_t i = 0; i < lid_obj_num; i++) {
        for (size_t j = 0; j < cam_obj_num; j++) {
            iou_mat(i, j) = compute_2d_iou(&pixel_lidar_objects->at(i),
                                           &pixel_camera_objects->at(j));
        }
    }
    return iou_mat;
}

/**
 * @brief 计算两个目标间的iou值
 * @param  pixel_lidar_object     激光雷达目标的二维像素坐标
 * @param  pixel_camera_object    摄像头目标的二维像素坐标
 * @return double 计算得到的iou值
 */
double compute_2d_iou(PixelLidarObjectRect *pixel_lidar_object,
                      PixelCameraObjectRect *pixel_camera_object) {
    int w = (std::min(pixel_lidar_object->right_btm.w,
                      pixel_camera_object->right_btm.w) -
             std::max(pixel_lidar_object->left_top.w,
                      pixel_camera_object->left_top.w));

    int h = (std::min(pixel_lidar_object->right_btm.h,
                      pixel_camera_object->right_btm.h) -
             std::max(pixel_lidar_object->left_top.h,
                      pixel_camera_object->left_top.h));
    int t = w * h;
    int a =
        ((pixel_lidar_object->right_btm.h - pixel_lidar_object->left_top.h) *
         (pixel_lidar_object->right_btm.w - pixel_lidar_object->left_top.w));
    int b =
        ((pixel_camera_object->right_btm.h - pixel_camera_object->left_top.h) *
         (pixel_camera_object->right_btm.w - pixel_camera_object->left_top.w));
    return (t / (a + b - t));
}

/**
 * @brief KM算法
 * @return 返回分配矩阵
 */
Eigen::MatrixXd KM_algorithm(Eigen::MatrixXd mat, double threshold) {
    // TODO:KM算法实现
    Eigen::MatrixXd ret(mat.size());
    double max = -1;
    int max_pos_row = -1;
    int max_pos_col = -1;
}

/**
 * @brief 卡尔曼滤波算法
 */
void kalman_filter() {
    // TODO:卡尔曼滤波算法实现
    ;
}
