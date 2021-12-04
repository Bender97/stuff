

#include "utility.h"

#include "pcl_ros/transforms.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>


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

        // initialization of tf2 listener (from transforms btw frames)
        tf_buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
        transform_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

        resetANDresizePointClouds();

        cellPointsBuckets.resize(grid.num_cells_per_side_squared);
        currveloBuckets.resize(grid.num_cells_per_side_squared);
        
        cellIsPredictable.resize(grid.num_cells_per_side_squared);

        enqueuedCloudsSizes.resize(cloudsQueue_fixed_size, 0);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLidarCloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIntegratedCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPredTravGrid;

    sensor_msgs::msg::PointCloud2 lidar_cloud_in_msg;
    sensor_msgs::msg::PointCloud2 lidar_cloud_in_map_msg;
    sensor_msgs::msg::PointCloud2 integrated_clouds_map_msg;
    sensor_msgs::msg::PointCloud2 integrated_clouds_lidar_msg;



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  _lidar_cloud_in;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  _lidar_cloud_map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr    _pred_trav_grid_map;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  _integrated_clouds_map;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  _integrated_clouds_lidar;
    pcl::PointCloud<PointType>::Ptr         _curr_trav_grid_map;
    pcl::PointCloud<PointType>::Ptr         _curr_trav_grid_lidar;
    
    // buckets use to sort points into cells of the grid based on their 3D coordinates
    std::vector<std::vector<pcl::PointXYZRGB *>> cellPointsBuckets;
    std::vector<std::vector<pcl::PointXYZRGB *>> currveloBuckets;
    
    // vector used to store the size of each cloud ( used in integration of subsequent clouds )
    std::vector<int> enqueuedCloudsSizes;

    // used to regulate the timestamp on each header packet published in this node
    std_msgs::msg::Header lidar_msg_header;

    // Transform Listener
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped transformStamped;

    int frame_cont = 0;

    cv::Ptr<cv::ml::SVM> svm;
    int features_num;
    
    std::vector<float> min, p2p;         // needed to normalize new data

    void resetANDresizePointClouds() {
        // RESET
        _lidar_cloud_in         .reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        _lidar_cloud_map        .reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        _integrated_clouds_map  .reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        _integrated_clouds_lidar.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        _pred_trav_grid_map   .reset(new pcl::PointCloud<PointType>());
        _curr_trav_grid_map         .reset(new pcl::PointCloud<PointType>());
        _curr_trav_grid_lidar   .reset(new pcl::PointCloud<PointType>());

        // RESIZE
        //_lidar_cloud_in->clear();
        _pred_trav_grid_map->points.resize(grid.num_cells_per_side_squared);
        _curr_trav_grid_map      ->points.resize(grid.num_cells_per_side_squared);
        _curr_trav_grid_lidar->points.resize(grid.num_cells_per_side_squared);
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
            _pred_trav_grid_map->points[idx].z = LOWEST_Z_VALUE;
            _curr_trav_grid_map->points[idx].z       = LOWEST_Z_VALUE;
            _curr_trav_grid_lidar->points[idx].z = LOWEST_Z_VALUE;
            _pred_trav_grid_map->points[idx].intensity = UNKNOWN_CELL_LABEL;
            _curr_trav_grid_map->points[idx].intensity       = UNKNOWN_CELL_LABEL;
            _curr_trav_grid_lidar->points[idx].intensity = UNKNOWN_CELL_LABEL;

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
                _pred_trav_grid_map->points[idx].x = x;
                _pred_trav_grid_map->points[idx].y = y;
                _curr_trav_grid_map->points[idx].x = x;
                _curr_trav_grid_map->points[idx].y = y;
            }
        }
    }

    void transformLidarCloudToMapFrame() {
        _lidar_cloud_in->header.frame_id = lidarFrame;
        pcl_ros::transformPointCloud(mapFrame,                  // target frame
                                     rclcpp::Time(0),
                                     *_lidar_cloud_in,          // cloud in
                                     lidarFrame,
                                     *_lidar_cloud_map,         // cloud out
                                     *tf_buffer_
        );
        RCLCPP_INFO(this->get_logger(), "transformed lidar_cloud_in to map frame");
    }

    void transformIntegratedCloudsToLidarFrame() {
        _integrated_clouds_map->header.frame_id = mapFrame;
        pcl_ros::transformPointCloud(lidarFrame,                  // target frame
                                     rclcpp::Time(0),
                                     *_integrated_clouds_map,          // cloud in
                                     mapFrame,
                                     *_integrated_clouds_lidar,         // cloud out
                                     *tf_buffer_
        );
        RCLCPP_INFO(this->get_logger(), "transformed integrated_clouds_map to lidar frame");
    }

    float getAvgElevationOfPointsInCell(std::vector<pcl::PointXYZRGB *> cell) {
        float z_sum = .0f;
        for (auto &point: cell) z_sum += point->z;
        return ( z_sum / ( (float) cell.size() ) );
    }

    void updateGridCellsElevation() {
        for (int cell_idx=0; cell_idx<grid.num_cells_per_side_squared; ++cell_idx) {
            /// if the cell has a suff num of points, set the avg elev to the pred cell
            if (cellPointsBuckets[cell_idx].size() >= (size_t) min_points_in_bucket)
                _pred_trav_grid_map->points[cell_idx].z = getAvgElevationOfPointsInCell(cellPointsBuckets[cell_idx]);
                /// else remove points which are unknown (only for visualization)
            else {
                _pred_trav_grid_map->points[cell_idx].intensity = UNKNOWN_CELL_LABEL;
                _pred_trav_grid_map->points[cell_idx].z         = HIGHEST_Z_VALUE;
            }
        }
    }

    void getGridInLidarFrame() {
        _curr_trav_grid_map->header.frame_id = mapFrame;
        pcl_ros::transformPointCloud(lidarFrame,
                                     rclcpp::Time(0),
                                     *_curr_trav_grid_map,
                                     mapFrame,
                                     *_curr_trav_grid_lidar,
                                     *tf_buffer_
                                     );
    }

    void publishClouds() {
        sensor_msgs::msg::PointCloud2 temp;

        if (pubPredTravGrid->get_subscription_count() != 0) {
            pcl::toROSMsg(*_pred_trav_grid_map, temp);
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

    void script() {


        // transform lidar cloud to mapFrame ( to integrate it with the previous clouds )
        transformLidarCloudToMapFrame();

        integrateClouds(_lidar_cloud_map, enqueuedCloudsSizes, _integrated_clouds_map);

        if ( frame_cont < cloudsQueue_fixed_size )     {
            RCLCPP_WARN_STREAM(this->get_logger(), "cannot predict yet or left image is empty -> Ignoring data (frame_cont: "
                            + std::to_string(frame_cont) + " / " + std::to_string(cloudsQueue_fixed_size) + ")");
            frame_cont++;
            return;
        }

        // transform integrated cloud to lidarFrame ( to exclude those points close to vehicle -- noise -- )
        transformIntegratedCloudsToLidarFrame();

        resetData();

        if (!updateLidarToMapTransform()) return;

        updateGridBottomRight_map();
        updateGridCellsCoordinates();

        GridManagement::sortPointsInGrid(_integrated_clouds_map, _integrated_clouds_lidar, cellPointsBuckets, gridBottomRight, grid);
        GridManagement::sortPointsInGrid(_lidar_cloud_map, _lidar_cloud_in, currveloBuckets, gridBottomRight, grid);

        for (int i=0; i<grid.num_cells_per_side_squared; ++i) {
            if (currveloBuckets[i].size()< (size_t) min_points_in_bucket_to_project_to_camera) {
                _curr_trav_grid_map->points[i].intensity = UNKNOWN_CELL_LABEL;
                _curr_trav_grid_map->points[i].z = HIGHEST_Z_VALUE;
            }
        }

        updateGridCellsElevation();

        getGridInLidarFrame();

        publishClouds();
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