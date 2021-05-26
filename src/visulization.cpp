#include "visulization.hpp"

// TODO
void Visualizer::open_camera(uint camera_index) {
    ;
    ;
}

void Visualizer::set_canvas_size(uint height, uint width) {
    this->canvas_height_ = height;
    this->canvas_width_ = width;
}

void Visualizer::input(cv::Mat *camera_image) {
    this->camera_image_ = camera_image;
}

void Visualizer::input(std::vector<RealLidarObject3d> *lidar_objects) {
    this->lidar_objects_ = lidar_objects;
}

void Visualizer::input(std::vector<PixelCameraObject> *camera_objetcs) {
    this->camera_objetcs_ = camera_objetcs;
}
