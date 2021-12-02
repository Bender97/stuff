
#pragma once

#ifndef TRAV_ANALYSIS_GRIDPARAMS_H
#define TRAV_ANALYSIS_GRIDPARAMS_H



struct GridParams {
    float   radius_of_attention;
    float   resolution;
    float   half_resolution;    // precalculated version of map_resolution to speed up the process (often used)
    float   resolution_squared; // precalculated version of map_resolution to speed up the process (often used)
    int     num_cells_per_side;          // the number of cells along a complete side of the grid
    float   half_num_cells_per_side;     // precalculated version of num_cells_per_side to speed up the process (often used)
    int     num_cells_per_side_squared;  // precalculated version of num_cells_per_side to speed up the process (often used)
    float   internal_resolution;
    int     internal_num_cells_per_side;
    int     internal_num_cells_per_side_squared;

    float vehicle_x_min, vehicle_x_max, vehicle_y_min, vehicle_y_max;
};

#endif //TRAV_ANALYSIS_GRIDPARAMS_H