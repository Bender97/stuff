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


#include "pcl_ros/transforms.hpp"
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/ml.hpp>


#include "Features.h"
#include "macro_utility.h"
#include "GridParams.h"
#include "Stats.h"
#include "DatasetStats.h"
//#include "IOUtils.h"
#include "CellManagement.h"
#include "GridManagement.h"
//#include "Camera.h"

#include <chrono>
#include <memory>
#include <string>

typedef pcl::PointXYZI  PointType;


class ParamServer : public rclcpp::Node {
protected:
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
    const std::string SVM_MODEL_PATH = "svmModelPath";

// Grid
    const std::string MIN_PTS_IN_BUCKET_GRID_PARAM = "minPointsInBucket";
    const std::string MIN_PTS_IN_BUCKET_TO_PROJECT_GRID_PARAM = "minPointsInBucketToProjectToCamera";
    const std::string MIN_NEIGHBORS_TO_PROP_LABEL_GRID_PARAM = "minNeighborsToPropagateLabel";
    const std::string PRED_LABEL_WEIGHT_GRID_PARAM = "predictedLabelWeight";
    const std::string RADIUS_GRID_PARAM = "radiusOfAttention";
    const std::string RESOLUTION_GRID_PARAM = "mapResolution";
    const std::string INTERNAL_RESOLUTION_GRID_PARAM = "internalCellResolution";

// Vehicle
    const std::string VEHICLE_X_MIN = "vehicleXMin";
    const std::string VEHICLE_X_MAX = "vehicleXMax";
    const std::string VEHICLE_Y_MIN = "vehicleYMin";
    const std::string VEHICLE_Y_MAX = "vehicleYMax";

// Features
    const std::string SINGLESCAN_BUCKETS_NUM = "curvityBucketsNum_singlescan";
    const std::string  MULTISCAN_BUCKETS_NUM = "curvityBucketsNum_multiscan";

// Training Parameters
    const std::string TRAINING_RATIO = "trainingRatio";
    const std::string KFOLDS = "kfolds";
    const std::string TERM_CRITERIA = "termCriteria";
    const std::string SKIP_COLS_WITH_INDEX = "skipColsWithIndex";
    const std::string USE_LINEAR_SVM = "useLinearSVM";
    const std::string USE_POLY_SVM = "usePolySVM";
    const std::string USE_RBF_SVM = "useRbfSVM";
    const std::string SAVE_LINEAR_SVM = "saveLinearSVM";
    const std::string SAVE_POLY_SVM = "savePolySVM";
    const std::string SAVE_RBF_SVM = "saveRbfSVM";
    const std::string LINEAR_SVM_NU = "linearSVM_Nu";
    const std::string LINEAR_SVM_GAMMA = "linearSVM_Gamma";
    const std::string POLY_SVM_NU = "polySVM_Nu";
    const std::string POLY_SVM_GAMMA = "polySVM_Gamma";
    const std::string POLY_SVM_DEGREE = "polySVM_Degree";
    const std::string RBF_SVM_NU = "rbfSVM_Nu";
    const std::string RBF_SVM_GAMMA = "rbfSVM_Gamma";

// Other Params
    const std::string CLOUDSQUEUE_FIXED_SIZE = "cloudsQueueFixedSize";

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
    std::string svm_model_path;

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

    // Features
    std::vector<short int> bucket;
    std::vector<short int> bucket_singlescan;
    std::vector<short int> bucket_multiscan;

    // SVM
    cv::Mat features_matrix, predictions_vector;
    std::vector<int> cellIsPredictable;

    // Traning parameters
    float training_ratio;
    int kfolds;
    float term_criteria;
//    std::vector<std::string> skip_cols_with_index__string;
    std::vector<double> skip_cols_with_index;
    bool use_linear_svm, use_poly_svm, use_rbf_svm;
    bool save_linear_svm, save_poly_svm, save_rbf_svm;
    std::vector<double> linearSVM_Nu, linearSVM_Gamma;
    std::vector<double> polySVM_Nu, polySVM_Gamma, polySVM_Degree;
    std::vector<double> rbfSVM_Nu, rbfSVM_Gamma;

    // Online System
    std::vector<int> feats_indexes;
    cv::Point3f scene_normal;
//
//
//    float global_map_side;
//    float max_point_height;
//
//    int timing_verbosity;

public:
    ParamServer() : Node("predicter") {

        declareParams();

        // Read parameters
        getParams();

        printParams();

    }


    void declareParams() {
        // Topics
        this->declare_parameter(LIDAR_TOPIC);
        this->declare_parameter(LEFT_IMAGE_TOPIC);
        this->declare_parameter(TRUE_TRAV_GRID_TOPIC);
        this->declare_parameter(PRED_TRAV_GRID_TOPIC);
        this->declare_parameter(INTEGRATED_CLOUD_TOPIC);

        // Frames
        this->declare_parameter(LIDAR_FRAME);
        this->declare_parameter(BASELINK_FRAME);
        this->declare_parameter(ODOMETRY_FRAME);
        this->declare_parameter(MAP_FRAME);

        // Paths
        this->declare_parameter(BASE_DIR_PATH);
        this->declare_parameter(DATA_PATH);
        this->declare_parameter(NORM_CONFIG_PATH);
        this->declare_parameter(LIN_SVM_PATH);
        this->declare_parameter(POL_SVM_PATH);
        this->declare_parameter(RBF_SVM_PATH);
        this->declare_parameter(SVM_MODEL_PATH);

        // Grid
        this->declare_parameter(MIN_PTS_IN_BUCKET_GRID_PARAM);
        this->declare_parameter(MIN_PTS_IN_BUCKET_TO_PROJECT_GRID_PARAM);
        this->declare_parameter(MIN_NEIGHBORS_TO_PROP_LABEL_GRID_PARAM);
        this->declare_parameter(PRED_LABEL_WEIGHT_GRID_PARAM);
        this->declare_parameter(RADIUS_GRID_PARAM);
        this->declare_parameter(RESOLUTION_GRID_PARAM);
        this->declare_parameter(INTERNAL_RESOLUTION_GRID_PARAM);

        // Vehicle
        this->declare_parameter(VEHICLE_X_MIN);
        this->declare_parameter(VEHICLE_X_MAX);
        this->declare_parameter(VEHICLE_Y_MIN);
        this->declare_parameter(VEHICLE_Y_MAX);

        // Features
        this->declare_parameter(SINGLESCAN_BUCKETS_NUM);
        this->declare_parameter( MULTISCAN_BUCKETS_NUM);

        // Training Parameters
        this->declare_parameter( TRAINING_RATIO);
        this->declare_parameter( KFOLDS);
        this->declare_parameter( TERM_CRITERIA);
        this->declare_parameter( SKIP_COLS_WITH_INDEX);
        this->declare_parameter( USE_LINEAR_SVM);
        this->declare_parameter( USE_POLY_SVM);
        this->declare_parameter( USE_RBF_SVM);
        this->declare_parameter( SAVE_LINEAR_SVM);
        this->declare_parameter( SAVE_POLY_SVM);
        this->declare_parameter( SAVE_RBF_SVM);
        this->declare_parameter( LINEAR_SVM_NU);
        this->declare_parameter( LINEAR_SVM_GAMMA);
        this->declare_parameter( POLY_SVM_NU);
        this->declare_parameter( POLY_SVM_GAMMA);
        this->declare_parameter( POLY_SVM_DEGREE);
        this->declare_parameter( RBF_SVM_NU);
        this->declare_parameter( RBF_SVM_GAMMA);

        // Other parameters
        this->declare_parameter(CLOUDSQUEUE_FIXED_SIZE);
    }

    void getParams() {

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
        if (!this->get_parameter(SVM_MODEL_PATH, svm_model_path)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", SVM_MODEL_PATH.c_str());
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
        if (!this->get_parameter(VEHICLE_X_MIN, grid.vehicle_x_min)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", VEHICLE_X_MIN.c_str());
        }
        if (!this->get_parameter(VEHICLE_X_MAX, grid.vehicle_x_max)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", VEHICLE_X_MAX.c_str());
        }
        if (!this->get_parameter(VEHICLE_Y_MIN, grid.vehicle_y_min)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", VEHICLE_Y_MIN.c_str());
        }
        if (!this->get_parameter(VEHICLE_Y_MAX, grid.vehicle_y_max)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", VEHICLE_Y_MAX.c_str());
        }

        // Features
        if (!this->get_parameter(SINGLESCAN_BUCKETS_NUM, curvity_buckets_num_singlescan)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", SINGLESCAN_BUCKETS_NUM.c_str());
        }
        if (!this->get_parameter(MULTISCAN_BUCKETS_NUM, curvity_buckets_num_multiscan)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", MULTISCAN_BUCKETS_NUM.c_str());
        }
        bucket_singlescan.resize(curvity_buckets_num_singlescan + 1);
        bucket_multiscan.resize(curvity_buckets_num_multiscan + 1);

        // Training Parameters
        if (!this->get_parameter(TRAINING_RATIO, training_ratio)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", TRAINING_RATIO.c_str());
        }
        if (!this->get_parameter(KFOLDS, kfolds)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", KFOLDS.c_str());
        }
        if (!this->get_parameter(TERM_CRITERIA, term_criteria)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", TERM_CRITERIA.c_str());
        }
        if (!this->get_parameter(SKIP_COLS_WITH_INDEX, skip_cols_with_index)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", SKIP_COLS_WITH_INDEX.c_str());
        }
//        for (auto &s : skip_cols_with_index__string) {
//            std::stringstream parser(s);
//            int x = 0;
//            parser >> x;
//            skip_cols_with_index.push_back(x);
//        }

        if (!this->get_parameter(USE_LINEAR_SVM, use_linear_svm)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", USE_LINEAR_SVM.c_str());
        }
        if (!this->get_parameter(USE_POLY_SVM, use_poly_svm)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", USE_POLY_SVM.c_str());
        }
        if (!this->get_parameter(USE_RBF_SVM, use_rbf_svm)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", USE_RBF_SVM.c_str());
        }
        if (!this->get_parameter(SAVE_LINEAR_SVM, save_linear_svm)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", SAVE_LINEAR_SVM.c_str());
        }
        if (!this->get_parameter(SAVE_POLY_SVM, save_poly_svm)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", SAVE_POLY_SVM.c_str());
        }
        if (!this->get_parameter(SAVE_RBF_SVM, save_rbf_svm)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", SAVE_RBF_SVM.c_str());
        }
        if (!this->get_parameter(LINEAR_SVM_NU, linearSVM_Nu)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", LINEAR_SVM_NU.c_str());
        }
        if (!this->get_parameter(LINEAR_SVM_GAMMA, linearSVM_Gamma)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", LINEAR_SVM_GAMMA.c_str());
        }
        if (!this->get_parameter(POLY_SVM_NU, polySVM_Nu)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", POLY_SVM_NU.c_str());
        }
        if (!this->get_parameter(POLY_SVM_GAMMA, polySVM_Gamma)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", POLY_SVM_GAMMA.c_str());
        }
        if (!this->get_parameter(POLY_SVM_DEGREE, polySVM_Degree)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", POLY_SVM_DEGREE.c_str());
        }
        if (!this->get_parameter(RBF_SVM_NU, rbfSVM_Nu)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", RBF_SVM_NU.c_str());
        }
        if (!this->get_parameter(RBF_SVM_GAMMA, rbfSVM_Gamma)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", RBF_SVM_GAMMA.c_str());
        }

        assert(training_ratio>.0f && training_ratio<1.0f);
        assert(kfolds>=2);

        // Other Parameters

        if (!this->get_parameter(CLOUDSQUEUE_FIXED_SIZE, cloudsQueue_fixed_size)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", CLOUDSQUEUE_FIXED_SIZE.c_str());
        }
    }

    void printParams() {
        // Topics
        RCLCPP_INFO(this->get_logger(), "%s: %s", LIDAR_TOPIC.c_str(), pointCloudTopic.c_str());

        RCLCPP_INFO(this->get_logger(), "%s: %s", LEFT_IMAGE_TOPIC.c_str(), leftImageTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", TRUE_TRAV_GRID_TOPIC.c_str(), trueTraversabilityTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", PRED_TRAV_GRID_TOPIC.c_str(), predictedTraversabilityTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", INTEGRATED_CLOUD_TOPIC.c_str(), integratedCloudsTopic.c_str());

        // Frames
        RCLCPP_INFO(this->get_logger(), "%s: %s", LIDAR_FRAME.c_str(), lidarFrame.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", BASELINK_FRAME.c_str(), baselinkFrame.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", ODOMETRY_FRAME.c_str(), odometryFrame.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", MAP_FRAME.c_str(), mapFrame.c_str());

        // Paths
        RCLCPP_INFO(this->get_logger(), "%s: %s", BASE_DIR_PATH.c_str(), base_dir_path.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", DATA_PATH.c_str(), data_path.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", NORM_CONFIG_PATH.c_str(), normalization_config_path.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", LIN_SVM_PATH.c_str(), linear_svm_path.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", POL_SVM_PATH.c_str(), poly_svm_path.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", RBF_SVM_PATH.c_str(), rbf_svm_path.c_str());
        RCLCPP_INFO(this->get_logger(), "%s: %s", SVM_MODEL_PATH.c_str(), svm_model_path.c_str());

        //Cameras

        // Grid
        RCLCPP_INFO(this->get_logger(), "%s: %d", MIN_PTS_IN_BUCKET_GRID_PARAM.c_str(), min_points_in_bucket);
        RCLCPP_INFO(this->get_logger(), "%s: %d", MIN_PTS_IN_BUCKET_TO_PROJECT_GRID_PARAM.c_str(), min_points_in_bucket_to_project_to_camera);
        RCLCPP_INFO(this->get_logger(), "%s: %d", MIN_NEIGHBORS_TO_PROP_LABEL_GRID_PARAM.c_str(), min_neighbors_to_propagate_labels);
        RCLCPP_INFO(this->get_logger(), "%s: %d", PRED_LABEL_WEIGHT_GRID_PARAM.c_str(), predicted_label_weight);
        RCLCPP_INFO(this->get_logger(), "%s: %f", RADIUS_GRID_PARAM.c_str(), grid.radius_of_attention);
        RCLCPP_INFO(this->get_logger(), "%s: %f", RESOLUTION_GRID_PARAM.c_str(), grid.resolution);
        RCLCPP_INFO(this->get_logger(), "%s: %f", INTERNAL_RESOLUTION_GRID_PARAM.c_str(), grid.internal_resolution);

        RCLCPP_INFO(this->get_logger(), "%s: %f", "grid.half_resolution", grid.half_resolution);
        RCLCPP_INFO(this->get_logger(), "%s: %d", "grid.resolution_squared", grid.resolution_squared);
        RCLCPP_INFO(this->get_logger(), "%s: %d", "grid.num_cells_per_side", grid.num_cells_per_side);
        RCLCPP_INFO(this->get_logger(), "%s: %f", "grid.half_num_cells_per_side", grid.half_num_cells_per_side);
        RCLCPP_INFO(this->get_logger(), "%s: %d", "grid.num_cells_per_side_squared", grid.num_cells_per_side_squared);
        RCLCPP_INFO(this->get_logger(), "%s: %d", "grid.internal_num_cells_per_side", grid.internal_num_cells_per_side);
        RCLCPP_INFO(this->get_logger(), "%s: %d", "grid.internal_num_cells_per_side_squared", grid.internal_num_cells_per_side_squared);
        RCLCPP_INFO(this->get_logger(), "%s: %f", "gridBottomRight.x", gridBottomRight.x);
        RCLCPP_INFO(this->get_logger(), "%s: %f", "gridBottomRight.y", gridBottomRight.y);


        // Vehicle
        RCLCPP_INFO(this->get_logger(), "%s: %f", VEHICLE_X_MIN.c_str(), grid.vehicle_x_min);
        RCLCPP_INFO(this->get_logger(), "%s: %f", VEHICLE_X_MAX.c_str(), grid.vehicle_x_max);
        RCLCPP_INFO(this->get_logger(), "%s: %f", VEHICLE_Y_MIN.c_str(), grid.vehicle_y_min);
        RCLCPP_INFO(this->get_logger(), "%s: %f", VEHICLE_Y_MAX.c_str(), grid.vehicle_y_max);

        // Features
        RCLCPP_INFO(this->get_logger(), "%s: %d", SINGLESCAN_BUCKETS_NUM.c_str(), curvity_buckets_num_singlescan);
        RCLCPP_INFO(this->get_logger(), "%s: %d", MULTISCAN_BUCKETS_NUM.c_str(), curvity_buckets_num_multiscan);

        // Training Parameters
        RCLCPP_INFO(this->get_logger(), "%s: %f", TRAINING_RATIO.c_str(), training_ratio);
        RCLCPP_INFO(this->get_logger(), "%s: %d", KFOLDS.c_str(), kfolds);
        RCLCPP_INFO(this->get_logger(), "%s: %f", TERM_CRITERIA.c_str(), term_criteria);
        std::stringstream s; s.clear(); for (auto elem : skip_cols_with_index) s << elem << ", ";
        RCLCPP_INFO(this->get_logger(), "%s: %s", SKIP_COLS_WITH_INDEX.c_str(), s.str().c_str());


        RCLCPP_INFO(this->get_logger(), "%s: %d", USE_LINEAR_SVM.c_str(), use_linear_svm);
        RCLCPP_INFO(this->get_logger(), "%s: %d", USE_POLY_SVM.c_str(), use_poly_svm);
        RCLCPP_INFO(this->get_logger(), "%s: %d", USE_RBF_SVM.c_str(), use_rbf_svm);
        RCLCPP_INFO(this->get_logger(), "%s: %d", SAVE_LINEAR_SVM.c_str(), save_linear_svm);
        RCLCPP_INFO(this->get_logger(), "%s: %d", SAVE_POLY_SVM.c_str(), save_poly_svm);
        RCLCPP_INFO(this->get_logger(), "%s: %d", SAVE_RBF_SVM.c_str(), save_rbf_svm);

        s.str(""); s.clear(); for (auto elem : linearSVM_Nu) s << elem << ", ";
        RCLCPP_INFO(this->get_logger(), "%s: %s", LINEAR_SVM_NU.c_str(), s.str().c_str());
        s.str(""); s.clear(); for (auto elem : linearSVM_Gamma) s << elem << ", ";
        RCLCPP_INFO(this->get_logger(), "%s: %s", LINEAR_SVM_GAMMA.c_str(), s.str().c_str());
        s.str(""); s.clear(); for (auto elem : polySVM_Nu) s << elem << ", ";
        RCLCPP_INFO(this->get_logger(), "%s: %s", POLY_SVM_NU.c_str(), s.str().c_str());
        s.str(""); s.clear(); for (auto elem : polySVM_Gamma) s << elem << ", ";
        RCLCPP_INFO(this->get_logger(), "%s: %s", POLY_SVM_GAMMA.c_str(), s.str().c_str());
        s.str(""); s.clear(); for (auto elem : polySVM_Degree) s << elem << ", ";
        RCLCPP_INFO(this->get_logger(), "%s: %s", POLY_SVM_DEGREE.c_str(), s.str().c_str());
        s.str(""); s.clear(); for (auto elem : rbfSVM_Nu) s << elem << ", ";
        RCLCPP_INFO(this->get_logger(), "%s: %s", RBF_SVM_NU.c_str(), s.str().c_str());
        s.str(""); s.clear(); for (auto elem : rbfSVM_Gamma) s << elem << ", ";
        RCLCPP_INFO(this->get_logger(), "%s: %s", RBF_SVM_GAMMA.c_str(), s.str().c_str());

        // Other Parameters
        RCLCPP_INFO(this->get_logger(), "%s: %d", CLOUDSQUEUE_FIXED_SIZE.c_str(), cloudsQueue_fixed_size);
    }


    bool columnIsRequested(int col) {
        for (const auto c: skip_cols_with_index) if (col==c) return false;
        return true;
    }


    void integrateClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _lidar_cloud_map,
                         std::vector<int> &enqueuedCloudsSizes,

                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _integrated_clouds_map) {


        size_t i;
        size_t final_size, old_data_size,  offset;

        /// compute final size of the integrated cloud
        final_size = (int) _lidar_cloud_map->points.size();
        for (i = 0; i < (size_t) cloudsQueue_fixed_size; ++i) final_size += enqueuedCloudsSizes[i];

        /// init temp data (cannot modify _integrated_clouds_map, we need that first part of the data!)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp;
        temp.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        temp->points.resize(final_size);

        /// copy new points up to velo size
        for (i = 0; i < _lidar_cloud_map->points.size(); ++i) temp->points[i] = _lidar_cloud_map->points[i];

        /// fill integrated cloud with the previous data
        offset = (int) _lidar_cloud_map->points.size();
        old_data_size = final_size - (int) _lidar_cloud_map->points.size();
        for (i = 0; i < old_data_size; ++i) temp->points[offset + i] = _integrated_clouds_map->points[i];

        /// shift clouds size
        for (i = cloudsQueue_fixed_size-1; i>0; i--) enqueuedCloudsSizes[i] = enqueuedCloudsSizes[i-1];
        if (cloudsQueue_fixed_size>=1) enqueuedCloudsSizes[0] = (int) _lidar_cloud_map->points.size();

        _integrated_clouds_map->points.resize(final_size);
        *_integrated_clouds_map = *temp;
        
        rclcpp::Time now = this->get_clock()->now();
        
        std::cout << "integrating end time " << now.seconds() << "." << now.nanoseconds() << std::endl;
    }


    /// calculate normal of the whole scene
    void computeScenePlaneNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Features &feature) {

        feature.matA1 = cv::Mat::zeros(cv::Size(3, 3), CV_32F);
        feature.matD1 = cv::Mat::zeros(cv::Size(1, 3), CV_32F);
        feature.matV1 = cv::Mat::zeros(cv::Size(3, 3), CV_32F);

        feature.computeCorrelationMatrixComponents(cloud);
        cv::eigen(feature.matA1, feature.matD1, feature.matV1);

        scene_normal.x = feature.matV1.at<float>(2, 0);
        scene_normal.y = feature.matV1.at<float>(2, 1);
        scene_normal.z = std::abs(feature.matV1.at<float>(2, 2));
    }

//    bool getCellFeatures(std::vector<pcl::PointXYZRGB *> &field_cell,
//                         Features &feature, bool singlescan_flag,
//                         PointType &belonging_cell_center,
//                         cv::Mat &hsv_image,
//                         PointType &cell,
//                         DatasetStats &dataset_stats) {
//
//        if (field_cell.size() < min_points_in_bucket) {
//            dataset_stats.empty_cell_amount++;
//            return false;
//        }
//
//        feature.label = (short int) belonging_cell_center.intensity;
//
//        if (feature.label==UNKNOWN_CELL_LABEL) {
//            dataset_stats.unknown_points_skipped++;
//            return false;
//        }
//
//        bucket = (singlescan_flag ? bucket_singlescan : bucket_multiscan);
//
//        if (!feature.computeGeometricalFeatures(field_cell, scene_normal, /*grid,*/ belonging_cell_center, bucket, curvity_buckets_num_singlescan)) {
//            if (feature.label == TRAV_CELL_LABEL)
//                dataset_stats.road_points_skipped_count++;
//            else
//                dataset_stats.non_road_points_skipped_count++;
//            return false;
//        }
//
//        if (feature.label == TRAV_CELL_LABEL)
//            dataset_stats.road_points_included_count++;
//        else
//            dataset_stats.non_road_points_included_count++;
//
//        cameraLeft.computeColorFeatures(feature, hsv_image, cell);
//
//        return true;
//    }

    bool getCellFeaturesPred(std::vector<pcl::PointXYZRGB *> &field_cell,
                             Features &feature, bool singlescan_flag,
                             PointType &belonging_cell_center,
//                             cv::Mat &hsv_image,
                             PointType &cell,
                             DatasetStats &dataset_stats) {

        if (field_cell.size() < (size_t) min_points_in_bucket) return false;

        bucket = (singlescan_flag ? bucket_singlescan : bucket_multiscan);

        if (!feature.computeGeometricalFeatures(field_cell, scene_normal,belonging_cell_center, curvity_buckets_num_singlescan)) {
            return false;
        }

//        cameraLeft.computeColorFeatures(feature, hsv_image, cell);

        return true;
    }
};

#endif //TRAV_ANALYSIS_UTILITY_H
