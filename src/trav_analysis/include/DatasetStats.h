#pragma once

#include <iostream>
#include <iomanip>

class DatasetStats {
public:
    int empty_cell_amount;
    int road_points_skipped_count;
    int road_points_included_count;
    int non_road_points_skipped_count;
    int non_road_points_included_count;
    int unknown_points_skipped;
    int tot_points;

    DatasetStats();

    DatasetStats(int fields_size);

    void resetStats(int fields_size);

    void printDatasetStats() const;

    void emptyCellAmount_increment();
    void roadPointsSkippedCount_increment();
    void roadPointsIncludedCount_increment();
    void nonRoadPointsSkippedCount_increment();
    void nonRoadPointsIncludedCount_increment();
    void unknownPointsSkipped_increment();
    void setTotPoints();

    float getUnknownRate() const;

    float getRoadIncludedRate() const;

    float getNonRoadIncludedRate() const;

    float getRoadSkippedRate() const;

    float getNonRoadSkippedRate() const;

    float getRoadIncludeTot() const;

    float getNonRoadIncludeTot() const;
};