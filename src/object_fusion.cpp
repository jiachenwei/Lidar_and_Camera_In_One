/**
 * @file object_fusion.cpp
 * @brief
 * @author Chenwei Jia (cwjia98@gmail.com)
 * @version 1.0
 * @date 2021-05-17
 */
#include "object_fusion.hpp"

#include <cmath>
#include <iostream>

// TODO:融合类实现
class Fusion {
   public:
    Fusion(/* args */);
    ~Fusion();

   private:
    uint pixel_image_height;
    uint pixel_image_width;
};

/**
 * @brief 计算2d_iou矩阵
 *
 */
Eigen::MatrixXd compute_2d_iou_matrix(
    std::vector<PixelLidarObject2d> *lidar_objects,
    std::vector<PixelCameraObject2d> *camera_objects) {
    uint lid_obj_num = lidar_objects->size();
    uint cam_obj_num = camera_objects->size();
    Eigen::MatrixXd iou_mat(lid_obj_num, cam_obj_num);
    for (size_t i = 0; i < lid_obj_num; i++) {
        for (size_t j = 0; j < cam_obj_num; j++) {
            iou_mat(i, j) =
                compute_2d_iou(&lidar_objects->at(i), &camera_objects->at(j));
        }
    }
    return iou_mat;
}

/**
 * @brief 计算两个目标间的iou值
 *
 */
double compute_2d_iou(PixelLidarObject2d *lidar_object,
                      PixelCameraObject2d *camera_object) {
    int w = (std::min(lidar_object->right_btm.w, camera_object->right_btm.w) -
             std::max(lidar_object->left_top.w, camera_object->left_top.w));

    int h = (std::min(lidar_object->right_btm.h, camera_object->right_btm.h) -
             std::max(lidar_object->left_top.h, camera_object->left_top.h));
    int t = w * h;
    int a = ((lidar_object->right_btm.h - lidar_object->left_top.h) *
             (lidar_object->right_btm.w - lidar_object->left_top.w));
    int b = ((camera_object->right_btm.h - camera_object->left_top.h) *
             (camera_object->right_btm.w - camera_object->left_top.w));
    return (t / (a + b - t));
}

/**
 * @brief KM算法，用来分配图像检测到的目标和激光雷达检测到的目标
 * @return 返回分配矩阵
 */
// TODO:KM算法实现
Eigen::MatrixXd KM_algorithm(Eigen::MatrixXd iou_mat, double threshold) {
    Eigen::MatrixXd ret(iou_mat.size());
    double max = -1;
    int max_pos_row = -1;
    int max_pos_col = -1;
}

/**
 * @brief 计算激光雷达的三维包围框在图像的二维平面上的投影的最小矩形
 *
 */
// TODO:三维投影实现
PixelLidarObject2d convert_lidar_object_3d_to_2d(
    RealLidarObject3d *lidar_obj_3d) {
    ;
    ;
}

/**
 * @brief 
 * @param  img              My Param doc
 * @return int 
 */
int camera_fun(cv::Mat *img);