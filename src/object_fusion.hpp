/**
 * @file lcio.hpp
 * @author Chenwei Jia (cwjia98@gmail.com)
 * @brief
 * 此头文件包含激光雷达与摄像头的融合方法与可视化方法，以及相关数据结构的定义
 * @note 此文件中，所有代码的x意为从原点出发指向正前方，y从右指向左，z从下指向上
 * @version 0.1
 * @date 2021-05-12
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <string>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "opencv2/core.hpp"

/**
 * @brief 三维空间的位置信息，单位m
 */
typedef struct RealPosition3d {
    double x;
    double y;
    double z;
} RealPosition3d;

/**
 * @brief 三维空间中的矩形尺寸类，单位m
 */
typedef struct RealRectSize3d {
    double x;
    double y;
    double z;
} RealRectSize3d;

/**
 * @brief 三维空间中物体旋转的四元数表示
 */
class RealOrientationQuat {
   private:
    double w_;
    double x_;
    double y_;
    double z_;

   public:
    RealOrientationQuat(double w, double x, double y, double z)
        : w_(w), x_(x), y_(y), z_(z){};
    ~RealOrientationQuat();
    /**
     * @brief 四元数转旋转矩阵
     * @return Eigen::Matrix3d 旋转矩阵
     */
    Eigen::Matrix3d to_rotation_matrix();

    /**
     * @brief 返回四元数
     * @return Eigen::Vector4d
     */
    Eigen::Vector4d quat();
};

/**
 * @brief 2d像素点位置
 */
typedef struct PixelPosition2d {
    int w;  // width
    int h;  // height
} PixelPosition2d;

/**
 * @brief 2d矩形框的像素尺寸，指名左上角与右下角位置
 */
typedef struct PixelRect2d {
    PixelPosition2d left_top;
    PixelPosition2d right_btm;
} PixelRect2d;

/**
 * @brief 真是坐标系下3d包围框的8个角点的结构体
 */
typedef struct RealCornerPoint3d {
    RealPosition3d front_left__btm;  //前左下
    RealPosition3d front_right_btm;  //前右下
    RealPosition3d front_left__top;  //前左上
    RealPosition3d front_right_top;  //前右上
    RealPosition3d rear__left__btm;  //后左下
    RealPosition3d rear__right_btm;  //后右下
    RealPosition3d rear__left__top;  //后左上
    RealPosition3d rear__right_top;  //后右上
} RealCornerPoint3d;

/**
 * @brief 激光雷达三维包围框的物理信息
 */
class RealLidarBoundingBox3d {
   private:
    RealPosition3d pos_;              // 3d包围框的中心位置
    RealRectSize3d size_;             // 3d包围框的尺寸
    RealOrientationQuat quat_;        // 3d包围框的旋转四元数
    RealCornerPoint3d corner_point_;  // 3d包围框的8个角点
    void compute_corner_point();

   public:
    /**
     * @brief Construct a new Real Lidar Bounding Box 3d object
     * @param  pos              中心位置
     * @param  size             尺寸
     * @param  quat             旋转四元数
     */
    RealLidarBoundingBox3d(RealPosition3d pos, RealRectSize3d size,
                           RealOrientationQuat quat)
        : pos_(pos), size_(size), quat_(quat) {
        compute_corner_point();
    };

    ~RealLidarBoundingBox3d();

    /**
     * @brief 返回8个角点的结构体
     * @return RealCornerPoint3d
     */
    RealCornerPoint3d corner_points() { return corner_point_; }

    /**
     * @brief 返回3d包围框的真实中心位置
     * @return RealPosition3d
     */
    RealPosition3d position() { return pos_; }

    /**
     * @brief 返回3d包围框的真实尺寸
     * @return RealRectSize3d
     */
    RealRectSize3d size() { return size_; }

    /**
     * @brief 返回3d包围框的旋转四元数
     * @return RealOrientationQuat
     */
    RealOrientationQuat quat() { return quat_; }
};

typedef RealLidarBoundingBox3d RealLidarObject3d;
typedef PixelRect2d PixelLidarObject2d;
typedef PixelRect2d PixelCameraObject2d;
