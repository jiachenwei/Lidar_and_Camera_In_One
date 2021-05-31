#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "core.hpp"

/**
 * @brief 可视化类，用来执行可视化的相关操作
 */
class Visualizer {
   public:
    Visualizer();
    Visualizer(uint camera_index, uint height, uint width);
    ~Visualizer();
    void open_camera(uint camera_index);
    void set_canvas_size(uint height, uint width);
    void input(cv::Mat *camera_image);
    void input(std::vector<RealLidarObject3d> *lidar_objects);
    void input(std::vector<PixelCameraObject> *camera_objetcs);
    void show();

   protected:
   private:
    uint canvas_height_;
    uint canvas_width_;
    uint camera_index_;

    cv::Mat *camera_image_;
    std::vector<RealLidarObject3d> *lidar_objects_;
    std::vector<PixelCameraObject> *camera_objetcs_;

    void draw_lidar_object_3d(std::vector<RealLidarObject3d> &lidar_objects);
    void draw_camera_object(std::vector<PixelCameraObject> &camera_objetcs);
};