
#include "DatasetStats.h"


DatasetStats::DatasetStats() {
    resetStats(0);
}

DatasetStats::DatasetStats(int fields_size) {
    resetStats(fields_size);
}

void DatasetStats::resetStats(int fields_size) {
    empty_cell_amount = 0;
    road_points_skipped_count = 0;
    road_points_included_count = 0;
    non_road_points_skipped_count = 0;
    non_road_points_included_count = 0;
    unknown_points_skipped = 0;
    tot_points = fields_size;
}

void DatasetStats::printDatasetStats() const {

    int length = (int) (std::to_string(tot_points).length());

    std::cout << " **** STATISTICS ****" << std::setprecision(3) << std::endl;
    std::cout << " ** unknown_points_skipped        : " << std::setw(length) << unknown_points_skipped           << " / " << tot_points << " (" << getUnknownRate()              << "% )" << std::endl;
    std::cout << " ** road_points_included_count    : " << std::setw(length) << road_points_included_count       << " / " << tot_points << " (" << getRoadIncludedRate()        << "% )" << " - (" << getRoadIncludeTot()        << "% )" << std::endl;
    std::cout << " ** non_road_points_included_count: " << std::setw(length) << non_road_points_included_count   << " / " << tot_points << " (" << getNonRoadIncludedRate()    << "% )" << " - (" << getNonRoadIncludeTot()    << "% )" << std::endl;
    std::cout << " ** road_points_skipped_count     : " << std::setw(length) << road_points_skipped_count        << " / " << tot_points << " (" << getRoadSkippedRate()         << "% )" << std::endl;
    std::cout << " ** non_road_points_skipped_count : " << std::setw(length) << non_road_points_skipped_count    << " / " << tot_points << " (" << getNonRoadSkippedRate()     << "% )" << std::endl;
    std::cout << std::endl;
}

void DatasetStats::emptyCellAmount_increment() { empty_cell_amount++; };
void DatasetStats::roadPointsSkippedCount_increment() { road_points_skipped_count++; };
void DatasetStats::roadPointsIncludedCount_increment() { road_points_included_count++; };
void DatasetStats::nonRoadPointsSkippedCount_increment() { non_road_points_skipped_count++; };
void DatasetStats::nonRoadPointsIncludedCount_increment() { non_road_points_included_count++; };
void DatasetStats::unknownPointsSkipped_increment() { unknown_points_skipped++; };
void DatasetStats::setTotPoints() { tot_points -= empty_cell_amount; }

float DatasetStats::getUnknownRate() const {
    if (tot_points>0)
        return (float) unknown_points_skipped * 100.0f / (float) tot_points;
    return .0f;
}

float DatasetStats::getRoadIncludedRate() const {
    if (tot_points>0)
        return (float) unknown_points_skipped * 100.0f / (float) tot_points;
    return .0f;
}

float DatasetStats::getNonRoadIncludedRate() const {
    if (tot_points>0)
        return (float) unknown_points_skipped * 100.0f / (float) tot_points;
    return .0f;
}

float DatasetStats::getRoadSkippedRate() const {
    if (tot_points>0)
        return (float) unknown_points_skipped * 100.0f / (float) tot_points;
    return .0f;
}

float DatasetStats::getNonRoadSkippedRate() const {
    if (tot_points>0)
        return (float) unknown_points_skipped * 100.0f / (float) tot_points;
    return .0f;
}

float DatasetStats::getRoadIncludeTot() const {
    int tot_included = road_points_included_count + non_road_points_included_count;
    if (tot_included>0)
        return (float) road_points_included_count * 100.0f / (float) (tot_included);
    return .0f;
}

float DatasetStats::getNonRoadIncludeTot() const {
    int tot_included = road_points_included_count + non_road_points_included_count;
    if (tot_included>0)
        return (float) non_road_points_included_count * 100.0f / (float) (tot_included);
    return .0f;
}