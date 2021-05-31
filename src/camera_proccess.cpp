#include "camera_proccess.hpp"

#include <stdio.h>

#include <iostream>
#include <string>

// TODO:YOLO目标检测
class CameraObjectDetector {
   private:
    float thresh_;
    char *class_name_file_addr_;
    char *cfg_file_addr_;
    char *weight_file_addr_;
    network *net_;
    std::vector<std::string> class_names_vector_;
    void image_convert(const cv::Mat *img, float *dst);
    void image_resize(float *src, float *dst, int src_width, int src_height,
                      int dst_width, int dst_height);

   public:
    CameraObjectDetector(char *cfg_file_addr, char *weight_file_addr,
                         char *class_name_file_addr);
    ~CameraObjectDetector();
    std::vector<std::pair<std::string, cv::Rect>> detect(cv::Mat *img);
};

// TODO:目标检测实现
CameraObjectDetector::CameraObjectDetector(char *cfg_file_addr,
                                           char *weight_file_addr,
                                           char *class_name_file_addr) {
    cfg_file_addr_ = cfg_file_addr;
    weight_file_addr_ = weight_file_addr;
    class_name_file_addr_ = class_name_file_addr;
    net_ = load_network(cfg_file_addr, weight_file_addr, 0);
    set_batch_network(net_, 1);
    std::ifstream class_name_file(class_name_file_addr_);
    if (class_name_file.is_open()) {
        std::string class_name = "";
        while (getline(class_name_file, class_name))
            class_names_vector_.push_back(class_name);
    }
}

CameraObjectDetector::~CameraObjectDetector() {}

void CameraObjectDetector::image_convert(const cv::Mat *img, float *dst) {
    uchar *data = img->data;
    int h = img->rows;
    int w = img->cols;
    int c = img->channels();

    for (int k = 0; k < c; ++k) {
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                dst[k * w * h + i * w + j] = data[(i * w + j) * c + k] / 255.;
            }
        }
    }
}

std::vector<std::pair<std::string, cv::Rect>> CameraObjectDetector::detect(
    cv::Mat *img) {
    cv::Mat rgb_img;
    cv::cvtColor(*img, rgb_img, cv::COLOR_BGR2RGB);
    float *src_img;
    size_t src_size = rgb_img.rows * rgb_img.cols * 3 * sizeof(float);
    src_img = (float *)malloc(src_size);
    image_convert(&rgb_img, src_img);

    float *resized_img;
    size_t new_size = net_->w * net_->h * 3 * sizeof(float);
    resized_img = (float *)malloc(new_size);
    image_resize(src_img, resized_img, img->cols, img->rows, net_->w,
                 net_->h);  //缩放图像

    network_predict(net_, resized_img);  //网络推理
    int boxes_num = 0;
    detection *dets = get_network_boxes(net_, rgb_img.cols, rgb_img.rows,
                                        thresh_, 0.5, 0, 1, &boxes_num);
}

/**
 * @brief 相机去畸变
 */
class CameraUndistortor {
   private:
    cv::Mat cameraMatrix_, newCameraMatrix_, distCoeffs_;
    cv::Mat map1_, map2_;
    cv::Size size_;
    int m1type_ = CV_32FC1;

   public:
    CameraUndistortor(cv::InputArray cameraMatrix, cv::InputArray distCoeffs,
                      cv::Size image_size);
    ~CameraUndistortor();

    void undisort(cv::Mat img, cv::Mat undistort_img);
};

/**
 * @brief Construct a new Camera Undistortor:: Camera Undistortor
 * object
 * @param  cameraMatrix     相机外参
 * @param  distCoeffs       相机内参
 * @param  image_size       图像尺寸
 */
CameraUndistortor::CameraUndistortor(cv::InputArray cameraMatrix,
                                     cv::InputArray distCoeffs,
                                     cv::Size image_size) {
    cameraMatrix_ = cv::Mat_<double>(3, 3) << cameraMatrix;
    distCoeffs_ = cv::Mat_<double>(5, 1) << distCoeffs;
    size_ = image_size;
    newCameraMatrix_ = getOptimalNewCameraMatrix(cameraMatrix_, distCoeffs_,
                                                 size_, 0, size_, 0);
    cv::initUndistortRectifyMap(cameraMatrix_, distCoeffs_, cv::Mat(),
                                newCameraMatrix_, size_, m1type_, map1_, map2_);
}

CameraUndistortor::~CameraUndistortor() {}

/**
 * @brief 对一个原始图像去畸变
 * @param  raw_img          原始图像
 * @param  undistort_img    去除畸变的图像
 */
void CameraUndistortor::undisort(cv::Mat raw_img, cv::Mat undistort_img) {
    remap(raw_img, undistort_img, map1_, map2_, cv::INTER_LINEAR);
}

///////////////////////////////////////////////////////////////////////////

void test_detector(char *datacfg, char *cfgfile, char *weightfile,
                   char *filename, float thresh, float hier_thresh,
                   char *outfile, int fullscreen) {
    list *options = read_data_cfg(datacfg);
    char *name_list = option_find_str(options, "names", "data/names.list");
    char **names = get_labels(name_list);

    image **alphabet = load_alphabet();
    network *net = load_network(cfgfile, weightfile, 0);
    set_batch_network(net, 1);
    srand(2222222);
    double time;
    char buff[256];
    char *input = buff;
    float nms = .45;
    while (1) {
        if (filename) {
            strncpy(input, filename, 256);
        } else {
            printf("Enter Image Path: ");
            fflush(stdout);
            input = fgets(input, 256, stdin);
            if (!input) return;
            strtok(input, "\n");
        }
        image im = load_image_color(input, 0, 0);
        image sized = letterbox_image(im, net->w, net->h);
        // image sized = resize_image(im, net->w, net->h);
        // image sized2 = resize_max(im, net->w);
        // image sized = crop_image(sized2, -((net->w - sized2.w)/2),
        // -((net->h
        // - sized2.h)/2), net->w, net->h); resize_network(net, sized.w,
        // sized.h);
        layer l = net->layers[net->n - 1];

        float *X = sized.data;
        time = what_time_is_it_now();
        network_predict(net, X);
        printf("%s: Predicted in %f seconds.\n", input,
               what_time_is_it_now() - time);
        int nboxes = 0;
        detection *dets = get_network_boxes(net, im.w, im.h, thresh,
                                            hier_thresh, 0, 1, &nboxes);
        // printf("%d\n", nboxes);
        // if (nms) do_nms_obj(boxes, probs, l.w*l.h*l.n, l.classes, nms);
        if (nms) do_nms_sort(dets, nboxes, l.classes, nms);
        draw_detections(im, dets, nboxes, thresh, names, alphabet, l.classes);
        free_detections(dets, nboxes);
        if (outfile) {
            save_image(im, outfile);
        } else {
            save_image(im, "predictions");
#ifdef OPENCV
            make_window("predictions", 512, 512, 0);
            show_image(im, "predictions", 0);
#endif
        }

        free_image(im);
        free_image(sized);
        if (filename) break;
    }
}