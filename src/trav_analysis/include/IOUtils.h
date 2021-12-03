
#ifndef TRAV_ANALYSIS_IOUTILS_H
#define TRAV_ANALYSIS_IOUTILS_H

#include "utility.h"
#include <boost/filesystem.hpp>

#include "Stats.h"

/**
 * Stats contains all the statistics produced to evaluate the training and testing performances of a
 * machine learning model. <br>
 * <br>
 * FP rate: how many non-traversable samples are classified as traversable.
 *          This is very dangerous and accident-prone. It should be as low as possible!
 *          It's also shown how many zeros are wrongly classified as ones (the FP number). <br>
 * FN rate: how many traversable samples are classified as non-traversable.
 *          This is not accident-prone, it is just too much cautious. <br>
 */
class IOUtils {
public:


    static bool checkAndCreateDir(std::string &path);
    static bool dirExists(std::string &path);

    static void checkPaths(std::string base_dir_path, rclcpp::Logger &logger);

    static void writeStatsToFile(std::ofstream &out_stats_file,
                                 Stats &stats,
                                 int frame_cont,
                                 uint32_t sec,
                                 uint32_t nsec,
                                 int64_t sim_duration) ;

    static void loadNormalizationConfig(std::string &normalization_config_path,
                                 int features_num,
                                 std::vector<float> &min,
                                 std::vector<float> &p2p);
};

#endif // TRAV_ANALYSIS_IOUTILS_H