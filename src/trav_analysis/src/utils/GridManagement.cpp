#include "GridManagement.h"

void GridManagement::sortPointsInGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudToSort,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudToSortExpressedInVeloFrame,
                      std::vector<std::vector<pcl::PointXYZRGB *>> &grid_buckets,
                                      cv::Point2f gridBottomRight,
                                      GridParams &grid) {
    Cell belonging_cell_center;
    int cell_idx;

    pcl::PointXYZRGB *p;
    pcl::PointXYZRGB *velo_p;

    for (size_t i=0; i<cloudToSort->points.size(); i++) {

        velo_p = &(cloudToSortExpressedInVeloFrame->points[i]);
//        if (std::abs(velo_p->z) > max_point_height) continue;
        if (isPointInsideVehicle(velo_p->x, velo_p->y, grid)) continue;

        p = &(cloudToSort->points[i]);
        getBelongingCell(p->x, p->y, gridBottomRight, grid, belonging_cell_center);

        if (belonging_cell_center.isInsideBounds(grid.num_cells_per_side)) {
            cell_idx = getIndexOfBelongingCellCenter(belonging_cell_center.row, belonging_cell_center.col, grid.num_cells_per_side);    // no check (already checked in isinsidebounds)
            // add the point to its cell
            grid_buckets[cell_idx].push_back(p);
            p->r = (53 * belonging_cell_center.row + 17  * belonging_cell_center.col) % 255;
            p->g = (23 * belonging_cell_center.row + 101 * belonging_cell_center.col) % 255;
            p->b = (91 * belonging_cell_center.row + 113 * belonging_cell_center.col) % 255;
        }
    }
}

void GridManagement::getGroundTruth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudToSort,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudToSortExpressedInVeloFrame,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr &gtTravGrid,
                    cv::Point2f gridBottomRight,
                    GridParams &grid) {
    Cell cell;
    PointType *belonging_cell_center;
    int cell_idx;

    pcl::PointXYZRGB *p;
    pcl::PointXYZRGB *velo_p;

    for (size_t i=0; i<cloudToSort->points.size(); i++) {

        velo_p = &(cloudToSortExpressedInVeloFrame->points[i]);
        //if (std::abs(velo_p->z) > max_point_height) continue;
        if (isPointInsideVehicle(velo_p->x, velo_p->y, grid)) continue;

        p = &(cloudToSort->points[i]);
        getBelongingCell(p->x, p->y, gridBottomRight, grid, cell);

        if (cell.isInsideBounds(grid.num_cells_per_side)) {

            cell_idx = getIndexOfBelongingCellCenter(cell.row, cell.col, grid.num_cells_per_side);    // no check (already checked in isinsidebounds)

            // make the pointer to point to its cell
            belonging_cell_center = &(gtTravGrid->points[cell_idx]);

            // update the cell's elevation according to the current elevation (keep the max value)
            if (p->z > belonging_cell_center->z) {
                belonging_cell_center->intensity = (POINTBELONGSTOROAD(p->r, p->g, p->b) ? TRAV_CELL_LABEL : NOT_TRAV_CELL_LABEL);
                belonging_cell_center->z = p->z;
            }
        }
    }
}



bool GridManagement::isPointInsideVehicle(float x, float y, GridParams &grid) {
    return x<grid.vehicle_x_max && x>grid.vehicle_x_min && y>grid.vehicle_y_min && y<grid.vehicle_y_max;
}

int GridManagement::getIndexOfBelongingCellCenter(int row, int col, int side) {
    return row * side + col;
}

void GridManagement::getBelongingCell(float x, float y, cv::Point2f gridBottomRight, GridParams &grid, Cell &cell) {
    int col = (int) roundf((x - gridBottomRight.x) / grid.resolution);
    int row = (int) roundf((y - gridBottomRight.y) / grid.resolution);
    cell.set(row, col);
}

