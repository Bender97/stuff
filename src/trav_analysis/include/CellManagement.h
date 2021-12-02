//
// Created by fusy on 22/07/21.
//

#pragma once

#ifndef TRAV_ANALYSIS_CELLMANAGEMENTUTILITY_H
#define TRAV_ANALYSIS_CELLMANAGEMENTUTILITY_H

#include <math.h>

class Cell{
public:
    int row;
    int col;

    Cell();


    template<typename T>
    void set(T row_, T col_) {
        row = (int) row_;
        col = (int) col_;
    }
    template<typename T>
    bool isOutOfBounds(T limit) {
        return row<0 || row>=limit || col<0 || col>=limit;
    }
    template<typename T>
    bool isInsideBounds(T limit) {
        return !(row<0 || row>=limit || col<0 || col>=limit);
    }
};

class InternalCell : public Cell {
public:

    float map_resolution;
    float half_map_resolution;
    float internal_cell_resolution;
    int internal_grid_side_length;

    InternalCell(float _map_resolution, float _internal_cell_resolution);

    template<typename CustomPoint, typename CustomPoint2>
    void setInternalRowAndCol(CustomPoint &point, CustomPoint2 &belonging_cell_center) {
        row = floor(((belonging_cell_center.y + half_map_resolution - point->y) / map_resolution) * internal_grid_side_length);
        col = floor(((belonging_cell_center.x + half_map_resolution - point->x) / map_resolution) * internal_grid_side_length);

        if (row==internal_grid_side_length && (fabs(belonging_cell_center.y - point->y) - half_map_resolution  <= 1e-5 )) {
            row=internal_grid_side_length-1;
        }
        if (col==internal_grid_side_length && (fabs(belonging_cell_center.x - point->x - half_map_resolution) <= 1e-5 )) {
            col=internal_grid_side_length-1;
        }
    }
    template<typename CustomPoint, typename CustomPoint2>
    int getInternalIndex(CustomPoint &point, CustomPoint2 &belonging_cell_center) {
        setInternalRowAndCol(point, belonging_cell_center);
        return row * internal_grid_side_length + col;
    }
};

#endif // TRAV_ANALYSIS_CELLMANAGEMENTUTILITY_H
