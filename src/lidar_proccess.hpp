#include <vector>

#include "core.hpp"

void draw_lidar_objects(cv::Mat img);

std::vector<PixelLidarObjectRect> get_pixel_lidar_objects_rect(
    std::vector<RealLidarObject3d> lidar_objects_3d);