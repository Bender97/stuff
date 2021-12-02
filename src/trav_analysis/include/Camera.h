#pragma once

#ifndef TRAV_ANALYSIS_CAMERA_H
#define TRAV_ANALYSIS_CAMERA_H

#include "Features.h"
#include "GridParams.h"


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "opencv2/highgui.hpp"
#include "macro_utility.h"
#include "opencv2/opencv.hpp"

class Camera {
public:

    double Tr00, Tr01, Tr02, Tr03;
    double Tr10, Tr11, Tr12, Tr13;
    double Tr20, Tr21, Tr22, Tr23;

    double P200, P201, P202, P203;
    double P210, P211, P212, P213;
    double P220, P221, P222, P223;

    cv::Point polygon_vertex[4];
    cv::Mat mask;
    float avg, sdev;
    int tot_pixels;
    pcl::PointXYZI *point;
    cv::Vec3b mask_triplet, img_triplet;
    cv::Mat filteredImage;

    float off_x[4], off_y[4];

    int min_row;
    int min_col;
    int max_row;
    int max_col;

    cv::Scalar trav_color, non_trav_color;

    Camera();

    Camera(std::vector<double> param_Tr, std::vector<double> param_P2, GridParams &grid);


    bool computeColorFeatures(Features &feature, cv::Mat &hsv_image, pcl::PointXYZI &cell) ;

    void projectPoint(float x, float y, float z, int i);

    int getMinRow();

    int getMinCol();

    int getMaxRow();

    int getMaxCol();

    bool computePolygonVertex(int i, pcl::PointXYZI &cell, int rows, int cols);

    /// SHOW with a CV window the projection of the grid onto the image, along with the corrsponding mask
    void showGrid(cv::Mat &left_image, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::vector<std::vector<pcl::PointXYZRGB *>> &buckets  );

};


#endif //TRAV_ANALYSIS_CAMERA_H