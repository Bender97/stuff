//
// Created by fusy on 30/11/21.
//

#pragma once

#ifndef TRAV_ANALYSIS_UTILITY_H
#define TRAV_ANALYSIS_UTILITY_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
//#include <pcl_ros/point_cloud.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>


//#include <pcl_ros/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/ml.hpp>


#include "macro_utility.h"
#include "GridParams.h"
//#include "Stats.h"
//#include "DatasetStats.h"
//#include "IOUtils.h"
#include "CellManagement.h"
#include "GridManagement.h"
//#include "Features.h"
//#include "Camera.h"

#include <chrono>
#include <memory>
#include <string>

typedef pcl::PointXYZI  PointType;

const std::string LIDAR_TOPIC = "pointCloudTopic";
const std::string LEFT_IMAGE_TOPIC = "leftImageTopic";
const std::string TRUE_TRAV_GRID_TOPIC = "trueTraversabilityTopic";
const std::string PRED_TRAV_GRID_TOPIC = "predictedTraversabilityTopic";
const std::string INTEGRATED_CLOUD_TOPIC = "integratedCloudsTopic";

// Frames
const std::string LIDAR_FRAME = "lidarFrame";
const std::string BASELINK_FRAME = "baselinkFrame";
const std::string ODOMETRY_FRAME = "odometryFrame";
const std::string MAP_FRAME = "mapFrame";

// Paths
const std::string BASE_DIR_PATH = "baseDirPath";
const std::string DATA_PATH = "dataPath";
const std::string NORM_CONFIG_PATH = "normalizationConfigPath";
const std::string LIN_SVM_PATH = "linearSVMPath";
const std::string POL_SVM_PATH = "polySVMPath";
const std::string RBF_SVM_PATH = "rbfSVMPath";

// Grid
const std::string MIN_PTS_IN_BUCKET_GRID_PARAM = "minPointsInBucket";
const std::string MIN_PTS_IN_BUCKET_TO_PROJECT_GRID_PARAM = "minPointsInBucketToProjectToCamera";
const std::string MIN_NEIGHBORS_TO_PROP_LABEL_GRID_PARAM = "minNeighborsToPropagateLabel";
const std::string PRED_LABEL_WEIGHT_GRID_PARAM = "predictedLabelWeight";
const std::string RADIUS_GRID_PARAM = "radiusOfAttention";
const std::string RESOLUTION_GRID_PARAM = "mapResolution";
const std::string INTERNAL_RESOLUTION_GRID_PARAM = "internalCellResolution";

class ParamServer : public rclcpp::Node {
protected:

    // Topics
    std::string pointCloudTopic;
    std::string leftImageTopic;
    std::string trueTraversabilityTopic;
    std::string predictedTraversabilityTopic;
    std::string integratedCloudsTopic;

    // Frames
    std::string lidarFrame;
    std::string baselinkFrame;
    std::string odometryFrame;
    std::string mapFrame;

    // Paths
    std::string base_dir_path;
    std::string data_path;
    std::string normalization_config_path;
    std::string linear_svm_path, poly_svm_path, rbf_svm_path;
    std::string out_stats_path;

    // Cameras
//    Camera cameraLeft;
//    bool show_grid_projection_on_image_flag;
//    // camera projection matrices
//    std::vector<double> param_P2;
//    std::vector<double> param_Tr;
//
//    // Grid
    GridParams grid;
    int min_points_in_bucket;
    int min_points_in_bucket_to_project_to_camera;
    int min_neighbors_to_propagate_labels;
    int predicted_label_weight;
    int curvity_buckets_num_singlescan, curvity_buckets_num_multiscan;
    int cloudsQueue_fixed_size;
    cv::Point2f gridBottomRight;
//
//    // Features
//    std::vector<short int> bucket;
//    std::vector<short int> bucket_singlescan;
//    std::vector<short int> bucket_multiscan;
//
    // SVM
    cv::Mat features_matrix, predictions_vector;
    std::vector<int> cellIsPredictable;
//
//    // Traning parameters
//    float training_ratio;
//    int kfolds;
//    float term_criteria;
//    std::vector<int> skip_cols_with_index;
//    bool use_linear_svm, use_poly_svm, use_rbf_svm;
//    bool save_linear_svm, save_poly_svm, save_rbf_svm;
//    std::vector<double> linearSVM_Nu, linearSVM_Gamma;
//    std::vector<double> polySVM_Nu, polySVM_Gamma, polySVM_Degree;
//    std::vector<double> rbfSVM_Nu, rbfSVM_Gamma;
//
//    // Online System
//    std::string svm_model_path;
//    std::vector<int> feats_indexes;
//    cv::Point3f scene_normal;
//
//
//    float global_map_side;
//    float max_point_height;
//
//    int timing_verbosity;

public:
    ParamServer() : Node("predicter") {
        this->declare_parameter(LIDAR_TOPIC);
        this->declare_parameter(LEFT_IMAGE_TOPIC);
        this->declare_parameter(TRUE_TRAV_GRID_TOPIC);
        this->declare_parameter(PRED_TRAV_GRID_TOPIC);
        this->declare_parameter(INTEGRATED_CLOUD_TOPIC);

        this->declare_parameter(LIDAR_FRAME);
        this->declare_parameter(BASELINK_FRAME);
        this->declare_parameter(ODOMETRY_FRAME);
        this->declare_parameter(MAP_FRAME);

        this->declare_parameter(BASE_DIR_PATH);
        this->declare_parameter(DATA_PATH);
        this->declare_parameter(NORM_CONFIG_PATH);
        this->declare_parameter(LIN_SVM_PATH);
        this->declare_parameter(POL_SVM_PATH);
        this->declare_parameter(RBF_SVM_PATH);

        this->declare_parameter(MIN_PTS_IN_BUCKET_GRID_PARAM);
        this->declare_parameter(MIN_PTS_IN_BUCKET_TO_PROJECT_GRID_PARAM);
        this->declare_parameter(MIN_NEIGHBORS_TO_PROP_LABEL_GRID_PARAM);
        this->declare_parameter(PRED_LABEL_WEIGHT_GRID_PARAM);
        this->declare_parameter(RADIUS_GRID_PARAM);
        this->declare_parameter(RESOLUTION_GRID_PARAM);
        this->declare_parameter(INTERNAL_RESOLUTION_GRID_PARAM);

        // Read parameters

        // Topics
        if (!this->get_parameter(LIDAR_TOPIC, pointCloudTopic)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", LIDAR_TOPIC.c_str());
        }
        if (!this->get_parameter(LEFT_IMAGE_TOPIC, leftImageTopic)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", LEFT_IMAGE_TOPIC.c_str());
        }
        if (!this->get_parameter(TRUE_TRAV_GRID_TOPIC, trueTraversabilityTopic)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", TRUE_TRAV_GRID_TOPIC.c_str());
        }
        if (!this->get_parameter(PRED_TRAV_GRID_TOPIC, predictedTraversabilityTopic)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PRED_TRAV_GRID_TOPIC.c_str());
        }
        if (!this->get_parameter(INTEGRATED_CLOUD_TOPIC, integratedCloudsTopic)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", INTEGRATED_CLOUD_TOPIC.c_str());
        }

        // Frames
        if (!this->get_parameter(LIDAR_FRAME, lidarFrame)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", LIDAR_FRAME.c_str());
        }
        if (!this->get_parameter(BASELINK_FRAME, baselinkFrame)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", BASELINK_FRAME.c_str());
        }
        if (!this->get_parameter(ODOMETRY_FRAME, odometryFrame)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", ODOMETRY_FRAME.c_str());
        }
        if (!this->get_parameter(MAP_FRAME, mapFrame)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", MAP_FRAME.c_str());
        }

        // Paths
        if (!this->get_parameter(BASE_DIR_PATH, base_dir_path)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", BASE_DIR_PATH.c_str());
        }
        if (!this->get_parameter(DATA_PATH, data_path)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", DATA_PATH.c_str());
        }
        if (!this->get_parameter(NORM_CONFIG_PATH, normalization_config_path)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", NORM_CONFIG_PATH.c_str());
        }
        if (!this->get_parameter(LIN_SVM_PATH, linear_svm_path)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", LIN_SVM_PATH.c_str());
        }
        if (!this->get_parameter(POL_SVM_PATH, poly_svm_path)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", POL_SVM_PATH.c_str());
        }
        if (!this->get_parameter(RBF_SVM_PATH, rbf_svm_path)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", RBF_SVM_PATH.c_str());
        }

        //Cameras

        // Grid
        if (!this->get_parameter(MIN_PTS_IN_BUCKET_GRID_PARAM, min_points_in_bucket)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", MIN_PTS_IN_BUCKET_GRID_PARAM.c_str());
        }
        if (!this->get_parameter(MIN_PTS_IN_BUCKET_TO_PROJECT_GRID_PARAM, min_points_in_bucket_to_project_to_camera)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", MIN_PTS_IN_BUCKET_TO_PROJECT_GRID_PARAM.c_str());
        }
        if (!this->get_parameter(MIN_NEIGHBORS_TO_PROP_LABEL_GRID_PARAM, min_neighbors_to_propagate_labels)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", MIN_NEIGHBORS_TO_PROP_LABEL_GRID_PARAM.c_str());
        }
        if (!this->get_parameter(PRED_LABEL_WEIGHT_GRID_PARAM, predicted_label_weight)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PRED_LABEL_WEIGHT_GRID_PARAM.c_str());
        }
        if (!this->get_parameter(RADIUS_GRID_PARAM, grid.radius_of_attention)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", RADIUS_GRID_PARAM.c_str());
        }
        if (!this->get_parameter(RESOLUTION_GRID_PARAM, grid.resolution)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", RESOLUTION_GRID_PARAM.c_str());
        }
        if (!this->get_parameter(INTERNAL_RESOLUTION_GRID_PARAM, grid.internal_resolution)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", INTERNAL_RESOLUTION_GRID_PARAM.c_str());
        }

        assert(grid.radius_of_attention>0);      // otherwise, trivial behaviour
        assert(grid.resolution>0);               // map_resolution is used as denominator
        grid.half_resolution                     = grid.resolution / 2.0f;
        grid.resolution_squared                  = NUM2SQ(grid.resolution);
        grid.num_cells_per_side                  = (int) ( 2.0f * grid.radius_of_attention / grid.resolution);
        grid.half_num_cells_per_side             = (float) grid.num_cells_per_side / 2.0f;
        grid.num_cells_per_side_squared          = (int) pow(grid.num_cells_per_side, 2);
        grid.internal_num_cells_per_side         = (int) ceilf(grid.resolution / grid.internal_resolution);
        grid.internal_num_cells_per_side_squared = NUM2SQ(grid.internal_num_cells_per_side);
//        Features::gridParams = &grid;
        gridBottomRight.x = (float) grid.half_num_cells_per_side * grid.resolution;
        gridBottomRight.y = gridBottomRight.x;

        // Vehicle

        // Features

        // Training Parameters

        // Other Parameters
    }
};

#endif //TRAV_ANALYSIS_UTILITY_H
