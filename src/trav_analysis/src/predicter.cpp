

#include "utility.h"

#include "pcl_ros/transforms.hpp"

class Predicter : public ParamServer {

public:
    Predicter() {

        // initialization of subscriber to Lidar cloud
        subLidarCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                pointCloudTopic, 10, std::bind(&Predicter::lidarCloudHandler, this, std::placeholders::_1)
        );

        // initialization of publisher of predicted traversability grid
        pubIntegratedCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                integratedCloudsTopic, 1
        );
        pubPredTravGrid = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                predictedTraversabilityTopic, 1
        );

        // initialization of tf2 listener (from transformations btw frames)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        initPCs();

        cellPointsBuckets.resize(grid.num_cells_per_side_squared);
        currveloBuckets.resize(grid.num_cells_per_side_squared);
        cellIsPredictable.resize(grid.num_cells_per_side_squared);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLidarCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIntegratedCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPredTravGrid;

    sensor_msgs::msg::PointCloud2 lidar_cloud_in_msg;
    sensor_msgs::msg::PointCloud2 lidar_cloud_in_map_msg;



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _lidar_cloud_in;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _lidar_cloud_map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _pred_trav_grid_lidar;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _integrated_clouds_map;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _integrated_clouds_lidar;

    std::vector<std::vector<pcl::PointXYZRGB *>> cellPointsBuckets;
    std::vector<std::vector<pcl::PointXYZRGB *>> currveloBuckets;
    std::vector<int> enqueuedCloudsSizes;

    std_msgs::msg::Header lidar_msg_header;

    // Transform Listener
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped transformStamped;

    cv::Ptr<cv::ml::SVM> svm;
    int features_num;

    tf2::Transform prova;

    std::vector<float> min, p2p;         // needed to normalize new data

    void initPCs() {
        // RESET
        _lidar_cloud_in         .reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        _lidar_cloud_map        .reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        _pred_trav_grid_lidar   .reset(new pcl::PointCloud<pcl::PointXYZI>());
        _integrated_clouds_map  .reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        _integrated_clouds_lidar.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        // RESIZE
        //_lidar_cloud_in->clear();
        _pred_trav_grid_lidar->points.resize(grid.num_cells_per_side_squared);
    }

    void lidarCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
        lidar_msg_header = laserCloudMsg->header;
        lidar_cloud_in_msg = *laserCloudMsg;

        pcl::fromROSMsg(*laserCloudMsg, *_lidar_cloud_in);
        std::cout << "points: " << _lidar_cloud_in->points.size() << std::endl;

        script();
    }

    void resetData() {
        for (int idx=0; idx<grid.num_cells_per_side_squared; ++idx) {
            _pred_trav_grid_lidar->points[idx].z       = LOWEST_Z_VALUE;
//            currTravGrid->points[idx].z       = LOWEST_Z_VALUE;
//            currTravGrid_lidar->points[idx].z = LOWEST_Z_VALUE;
            _pred_trav_grid_lidar->points[idx].intensity       = UNKNOWN_CELL_LABEL;
//            currTravGrid->points[idx].intensity       = UNKNOWN_CELL_LABEL;
//            currTravGrid_lidar->points[idx].intensity = UNKNOWN_CELL_LABEL;

            cellIsPredictable[idx] = 0;

            // clear each grid "linked list", in order to discriminate empty grids
            cellPointsBuckets[idx].clear();
            currveloBuckets[idx].clear();
        }

        features_matrix = cv::Mat::zeros(grid.num_cells_per_side_squared, features_num, CV_32F);
        predictions_vector = cv::Mat::zeros(grid.num_cells_per_side_squared, 1, CV_32FC1);

    }

    bool updateLidarToMapTransform() {
        try {
            transformStamped = tf_buffer_->lookupTransform(
                    lidarFrame, mapFrame,
                    tf2::TimePointZero );
        } catch (tf2::TransformException & ex) {
            RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    lidarFrame.c_str(), mapFrame.c_str(), ex.what());
            return false;
        }
        return true;
    }

    bool updateMapToLidarTransform() {
        try {
            transformStamped = tf_buffer_->lookupTransform(
                    mapFrame, lidarFrame,
                    tf2::TimePointZero );
        } catch (tf2::TransformException & ex) {
            RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    lidarFrame.c_str(), mapFrame.c_str(), ex.what());
            return false;
        }
        return true;
    }

    void updateGridBottomRight_map() {
        gridBottomRight.x = (float) transformStamped.transform.translation.x - grid.half_num_cells_per_side * grid.resolution - grid.half_resolution;
        gridBottomRight.y = (float) transformStamped.transform.translation.y - grid.half_num_cells_per_side * grid.resolution - grid.half_resolution;
    }

    void updateGridCellsCoordinates() {
        float x, y;
        for (int col=0; col<grid.num_cells_per_side; ++col) {
            for (int row = 0; row < grid.num_cells_per_side; ++row) {
                int idx = row + col* grid.num_cells_per_side;
                x = (float) col * grid.resolution + gridBottomRight.x;
                y = (float) row * grid.resolution + gridBottomRight.y;
                _pred_trav_grid_lidar->points[idx].x = x;
                _pred_trav_grid_lidar->points[idx].y = y;
                _pred_trav_grid_lidar->points[idx].z = 0;
                _pred_trav_grid_lidar->points[idx].intensity = 100;
            }
        }
    }

    void script() {

        // transform lidar cloud to mapFrame ( to integrate it with the previous clouds )
        _lidar_cloud_in->header.frame_id = lidarFrame;


        pcl_ros::transformPointCloud(mapFrame,                  // target frame
                                     lidar_cloud_in_msg,          // cloud in
                                     lidar_cloud_in_map_msg,         // cloud out
                                     *tf_buffer_
                                     );
        pcl::fromROSMsg(lidar_cloud_in_map_msg, *_lidar_cloud_map);
        RCLCPP_INFO(this->get_logger(), "transformed lidar cloud in to map frame");


        updateLidarToMapTransform();
        updateGridBottomRight_map();
        updateGridCellsCoordinates();

        *_integrated_clouds_map = *_lidar_cloud_map;
        *_integrated_clouds_lidar = *_lidar_cloud_map;

//        GridManagement::sortPointsInGrid(_integrated_clouds_map, _integrated_clouds_lidar, cellPointsBuckets, gridBottomRight, grid);
        GridManagement::sortPointsInGrid(_lidar_cloud_map, _integrated_clouds_lidar, cellPointsBuckets, gridBottomRight, grid);

        publishClouds();
    }

    void publishClouds() {
        sensor_msgs::msg::PointCloud2 temp;

        if (pubPredTravGrid->get_subscription_count() != 0) {
            pcl::toROSMsg(*_pred_trav_grid_lidar, temp);
            temp.header.stamp = lidar_msg_header.stamp;
            temp.header.frame_id = mapFrame;
            pubPredTravGrid->publish(temp);
        }

        if (pubIntegratedCloud->get_subscription_count() != 0) {
            pcl::toROSMsg(*_integrated_clouds_map, temp);
            temp.header.stamp = lidar_msg_header.stamp;
            temp.header.frame_id = mapFrame;
            pubIntegratedCloud->publish(temp);
        }
    }

    void loadSVM() {

        /// LOAD SVM
//        svm = cv::ml::SVM::load(svm_model_path);
//        assert(svm->isTrained());
//
//        features_num = svm->getVarCount();
//
//        // stack the indexes of the features that svm will use
//        for (int i=0; i<22; ++i) {
//            if(columnIsRequested(i))
//                feats_indexes.push_back(i);
//        }
    }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Predicter>());
    rclcpp::shutdown();
    return 0;
}