//
// Created by fusy on 22/07/21.
//

#pragma once

#ifndef TRAV_ANALYSIS_GRIDMANAGEMENTUTILITY_H
#define TRAV_ANALYSIS_GRIDMANAGEMENTUTILITY_H

#include "utility.h"

class GridManagement {
public:
    static void sortPointsInGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudToSort,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudToSortExpressedInVeloFrame,
                          std::vector<std::vector<pcl::PointXYZRGB *>> &grid_buckets,
                                 cv::Point2f gridBottomRight,
                                 GridParams &grid);

    static void getGroundTruth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudToSort,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudToSortExpressedInVeloFrame,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr &gtTravGrid,
                               cv::Point2f gridBottomRight,
                               GridParams &grid);



    static int getIndexOfBelongingCellCenter(int row, int col, int side);

    static bool isPointInsideVehicle(float x, float y, GridParams &grid);

    static void getBelongingCell(float x, float y, cv::Point2f gridBottomRight, GridParams &grid, Cell &cell);
};

#endif // TRAV_ANALYSIS_GRIDMANAGEMENTUTILITY_H
