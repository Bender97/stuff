#include "../../include/Camera.h"

Camera::Camera() {};

Camera::Camera(std::vector<double> param_Tr, std::vector<double> param_P2, GridParams &grid) {
    Tr00 = param_Tr[0]; Tr01 = param_Tr[1]; Tr02 = param_Tr[2];  Tr03 = param_Tr[3];
    Tr10 = param_Tr[4]; Tr11 = param_Tr[5]; Tr12 = param_Tr[6];  Tr13 = param_Tr[7];
    Tr20 = param_Tr[8]; Tr21 = param_Tr[9]; Tr22 = param_Tr[10]; Tr23 = param_Tr[11];

    P200 = param_P2[0]; P201 = param_P2[1]; P202 = param_P2[2];  P203 = param_P2[3];
    P210 = param_P2[4]; P211 = param_P2[5]; P212 = param_P2[6];  P213 = param_P2[7];
    P220 = param_P2[8]; P221 = param_P2[9]; P222 = param_P2[10]; P223 = param_P2[11];

    off_x[0] =  0.2;    off_y[0] =  0.2;
    off_x[1] =  0.2;    off_y[1] = -0.2;
    off_x[2] = -0.2;    off_y[2] = -0.2;
    off_x[3] = -0.2;    off_y[3] =  0.2;

    trav_color     = cv::Scalar(255, 255, 255);
    non_trav_color = cv::Scalar(0, 0, 255);
}




bool Camera::computeColorFeatures(Features &feature, cv::Mat &hsv_image, pcl::PointXYZI &cell) {

//    cv::Point polygon_vertex[4];
//    cv::Mat mask;
    float avg, sdev;
    int tot_pixels;
//    pcl::PointXYZI *point;
//    cv::Vec3b mask_triplet, img_triplet;
//    cv::Mat filteredImage;
//    float off_x[4];
//    float off_y[4];
    int min_row;
    int min_col;
    int max_row;
    int max_col;
    float color_hsv[3];

    /// initialize default values
    feature.color_avg  = -1;
    feature.color_sdev = -1;

    /// only points known and appearing in front of the sensor are to be considered (just a speed up check)
    if (cell.x < 0 || cell.intensity < 0) return false;

    /// project each vertex and check that it appears in the image view
    for (int i=0; i<4; i++) if (!computePolygonVertex(i, cell, hsv_image.rows, hsv_image.cols)) return false;

    /// init params
    avg = .0f; sdev = .0f; tot_pixels = 0;
//    filteredImage = cv::Mat::zeros(hsv_image.size(), hsv_image.type());
    for (int hsv_idx=0; hsv_idx<3; ++hsv_idx) color_hsv[hsv_idx] = .0f;

    /// build the mask of the polygon       TODO: substitute with odd raycasting for polygon check
    mask = cv::Mat::zeros(hsv_image.rows, hsv_image.cols, hsv_image.type());

    cv::fillConvexPoly( mask, polygon_vertex, 4, cv::Scalar(255, 255, 255) );

    min_row = getMinRow(); min_col = getMinCol();
    max_row = getMaxRow(); max_col = getMaxCol();

//    std::cout << "off_x[0]: " << off_x[0] << " off_x: " << off_x[1] << " off_x: " << off_x[2] << " off_x: " << off_x[3] << std::endl;
//    std::cout << "off_y[0]: " << off_y[0] << " off_y: " << off_y[1] << " off_y: " << off_y[2] << " off_y: " << off_y[3] << std::endl;
//    std::cout << "pv0: " << polygon_vertex[0] << " pv1: " << polygon_vertex[1] << " pv2: " << polygon_vertex[2] << " pv3: " << polygon_vertex[3] << std::endl;
//    std::cout << "minrow: " << min_row << " max_row: " << max_row << " min_col: " << min_col << " max_col: " << max_col << std::endl;

    for (int r = min_row; r<=max_row; r++) {
        for (int c = min_col; c<=max_col; c++) {
            mask_triplet = mask.at<cv::Vec3b>(r, c);
            if (mask_triplet[0]==255) {
                img_triplet = hsv_image.at<cv::Vec3b>(r, c);
                avg += (float) img_triplet[0];
                for (int hsv_idx=0; hsv_idx<3; ++hsv_idx) color_hsv[hsv_idx] += (float) img_triplet[hsv_idx];
                tot_pixels ++;
            }
        }
    }

    avg /= (float) tot_pixels;

    for (int hsv_idx=0; hsv_idx<3; ++hsv_idx) color_hsv[hsv_idx] /= (float) tot_pixels;

//    for (int r = min_row; r<=max_row; r++) {
//        for (int c = min_col; c<=max_col; c++) {
//            mask_triplet = mask.at<cv::Vec3b>(r, c);
//            if (mask_triplet[0]==255) {
//                img_triplet = hsv_image.at<cv::Vec3b>(r, c);
//                sdev += ((float) img_triplet[0] - avg)*((float) img_triplet[0] - avg);
//                std::cout << ((float) img_triplet[0] - avg)*((float) img_triplet[0] - avg) << std::endl;
//            }
//        }
//    }


    for (int r = min_row; r<=max_row; r++) {
        for (int c = min_col; c<=max_col; c++) {
            mask_triplet = mask.at<cv::Vec3b>(r, c);
            if (mask_triplet[0]==255) {
                img_triplet = hsv_image.at<cv::Vec3b>(r, c);
                sdev += ((float) img_triplet[0] - avg)*((float) img_triplet[0] - avg);
//                std::cout << ((float) img_triplet[0] - avg)*((float) img_triplet[0] - avg) << std::endl;
            }
        }
    }

//    std::cout << "sdev: " << sdev << (sdev>=0 ? " %%%%%%%%%%%%" : "") << std::endl;

    sdev /= (float) tot_pixels;
    if (sdev>1000) sdev = 1000;
    feature.color_avg  = avg;
    feature.color_sdev = sdev;

    for (int hsv_idx=0; hsv_idx<3; ++hsv_idx) feature.color_hsv[hsv_idx] = color_hsv[hsv_idx];

    return true;
}



void Camera::projectPoint(float x, float y, float z, int i) {
    float xt, yt, zt; // the 3D velodyne point projected to the 3D camera frame
    float u, v, w; // the projection of the 3D point to the 2D camera frame (not scaled)
    // the (scaled) projection of the 3D point to the 2D camera frame

    // project the point to the 3D camera frame using the transformation matrix of the camera
    xt = Tr00*x + Tr01*y + Tr02*z + Tr03;
    yt = Tr10*x + Tr11*y + Tr12*z + Tr13;
    zt = Tr20*x + Tr21*y + Tr22*z + Tr23;

    // project the point to the 2D camera frame using the camera projection matrix
    u = P200*xt + P201*yt + P202*zt + P203;
    v = P210*xt + P211*yt + P212*zt + P213;
    w = P220*xt + P221*yt + P222*zt + P223;

    // scale the pixel
    polygon_vertex[i].x = std::roundf(u / w);
    polygon_vertex[i].y = std::roundf(v / w);
}

int Camera::getMinRow() {
    int min = 100000;
    if (polygon_vertex[0].y < min) min = polygon_vertex[0].y;
    if (polygon_vertex[1].y < min) min = polygon_vertex[1].y;
    if (polygon_vertex[2].y < min) min = polygon_vertex[2].y;
    if (polygon_vertex[3].y < min) min = polygon_vertex[3].y;
    return min;
}

int Camera::getMinCol() {
    int min = 100000;
    if (polygon_vertex[0].x < min) min = polygon_vertex[0].x;
    if (polygon_vertex[1].x < min) min = polygon_vertex[1].x;
    if (polygon_vertex[2].x < min) min = polygon_vertex[2].x;
    if (polygon_vertex[3].x < min) min = polygon_vertex[3].x;
    return min;
}

int Camera::getMaxRow() {
    int max = -1;
    if (polygon_vertex[0].y > max) max = polygon_vertex[0].y;
    if (polygon_vertex[1].y > max) max = polygon_vertex[1].y;
    if (polygon_vertex[2].y > max) max = polygon_vertex[2].y;
    if (polygon_vertex[3].y > max) max = polygon_vertex[3].y;
    return max;
}

int Camera::getMaxCol() {
    int max = -1;
    if (polygon_vertex[0].x > max) max = polygon_vertex[0].x;
    if (polygon_vertex[1].x > max) max = polygon_vertex[1].x;
    if (polygon_vertex[2].x > max) max = polygon_vertex[2].x;
    if (polygon_vertex[3].x > max) max = polygon_vertex[3].x;
    return max;
}

bool Camera::computePolygonVertex(int i, pcl::PointXYZI &cell, int rows, int cols) {
    projectPoint(cell.x + off_x[i], cell.y + off_y[i], cell.z, i);
    if (polygon_vertex[i].x < 0 || polygon_vertex[i].x >= cols || polygon_vertex[i].y < 0 || polygon_vertex[i].y >= rows) return false;
    return true;
}



/// SHOW with a CV window the projection of the grid onto the image, along with the corrsponding mask
void Camera::showGrid(cv::Mat &image, pcl::PointCloud<pcl::PointXYZI>::Ptr &grid, std::vector<std::vector<pcl::PointXYZRGB *>> &buckets ) {
    cv::Mat leftImage_grid_projection = image.clone();
    cv::Mat leftImage_hsv;
    cv::cvtColor(leftImage_grid_projection, leftImage_hsv, cv::COLOR_BGR2HSV);
    pcl::PointXYZI cell;
    int i, idx;
    int rows = leftImage_grid_projection.rows;
    int cols = leftImage_grid_projection.cols;

    float color_hsv[3];

    bool flag;

    for ( idx = 0; idx < grid->points.size(); ++idx ) {

        cell = grid->points[idx];

        if ( cell.x < 0 || cell.intensity < 0 ) continue;

        /// project each vertex and check that it appears in the image view
        flag = false;
        for ( i=0; i<4; ++i ) {
            if (!computePolygonVertex(i, cell, rows, cols)) { flag = true; break; };
        }
        if (flag) continue;

        /// init params
        avg = .0f; sdev = .0f; tot_pixels = 0;
        filteredImage = cv::Mat::zeros(leftImage_hsv.size(), leftImage_hsv.type());

        for (int hsv_idx=0; hsv_idx<3; ++hsv_idx) color_hsv[hsv_idx] = .0f;

        /// build the mask of the polygon       TODO: substitute with odd raycasting for polygon check
        mask = cv::Mat::zeros(leftImage_hsv.rows, leftImage_hsv.cols, leftImage_hsv.type());

        cv::fillConvexPoly( mask, polygon_vertex, 4, cv::Scalar(255, 255, 255) );

        min_row = getMinRow(); min_col = getMinCol();
        max_row = getMaxRow(); max_col = getMaxCol();

        for (int r = min_row; r<=max_row; r++) {
            for (int c = min_col; c<=max_col; c++) {
                mask_triplet = mask.at<cv::Vec3b>(r, c);
                if (mask_triplet[0]==255) {
                    img_triplet = leftImage_hsv.at<cv::Vec3b>(r, c);
                    avg += (float) img_triplet[0];
                    for (int hsv_idx=0; hsv_idx<3; ++hsv_idx) color_hsv[hsv_idx] += (float) img_triplet[hsv_idx];
                    tot_pixels ++;
                }
            }
        }

        avg /= (float) tot_pixels;

        for (int hsv_idx=0; hsv_idx<3; ++hsv_idx) color_hsv[hsv_idx] /= (float) tot_pixels;

        if (grid->points[idx].intensity == 0) {
            for ( i=0; i<4; ++i )
                cv::line(leftImage_grid_projection, polygon_vertex[i], polygon_vertex[(i+1)%4], non_trav_color);
        }
        else {
            for ( i=0; i<4; ++i )
                cv::line(leftImage_grid_projection, polygon_vertex[i], polygon_vertex[(i+1)%4], trav_color);
        }

        grid->points[idx].intensity = avg / 200.0f;

    }

    cv::imshow("left_image_grid projection", leftImage_grid_projection);
    cv::waitKey(1);

}