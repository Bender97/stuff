#include "CellManagement.h"

Cell::Cell() {row=0; col=0;}

InternalCell::InternalCell(float _map_resolution, float _internal_cell_resolution) : Cell(){
    map_resolution = _map_resolution;
    internal_cell_resolution = _internal_cell_resolution;
    half_map_resolution = map_resolution/2.0f;
    internal_grid_side_length = (int) ceilf(map_resolution / internal_cell_resolution);
}

