
#pragma once

#ifndef TRAV_ANALYSIS_FEATURES_H
#define TRAV_ANALYSIS_FEATURES_H

#include "utility.h"

class Features {
public:
    float linearity;
    float planarity;
    float sphericity;
    float omnivariance;
    float anisotropy;
    float eigenentropy;
    float sum_of_eigenvalues;
    float curvature;

    float angle;
    float goodness_of_fit;
    float roughness;
    float average_elevation;

    cv::Point3f normal_vector;
    // cv::Point3f sum_of_vectors;                  // not implemented as I use grids
    float unevenness;
    float surface_density;

    float z_diff;
    float internal_density;        // density calculated using only occupied cells
    float curvity;

    float color_avg;
    float color_sdev;

    float color_hsv[3];

    short int label;

    static GridParams *gridParams;

//    float max_magn;
//    float min_magn;

    float cx, cy, cz;
    size_t numpoints;
    float a11, a12, a13, a22, a23, a33;
    cv::Mat matA1;
    cv::Mat matD1;
    cv::Mat matV1;
    float d1, d2, d3;

    void reset();

    template<typename CustomPoint>
    void computeCorrelationMatrixComponents(std::vector<CustomPoint *> cell);

    void computeCorrelationMatrixComponents(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cell);

    bool computeGeometricalFeatures( const std::vector<pcl::PointXYZRGB *> &points_in_bucket,
                                               cv::Point3f &scene_normal,
//                                               GridParams &grid,
                                               pcl::PointXYZI &belonging_cell_center,
                                               int numbuckets);

};


#endif //TRAV_ANALYSIS_FEATURES_H