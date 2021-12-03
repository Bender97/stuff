
#include <utility.h>
#include "IOUtils.h"


bool IOUtils::checkAndCreateDir(std::string &path) {
    if (!boost::filesystem::exists(path))
        return boost::filesystem::create_directory(path);
    return true;
}
bool IOUtils::dirExists(std::string &path) {
    return boost::filesystem::exists(path);
}

void IOUtils::checkPaths(std::string base_dir_path, rclcpp::Logger &logger) {
    assert(base_dir_path.length()>0);
    if (base_dir_path[0]!='/') {        //no absolute value
        checkAndCreateDir(base_dir_path);
    }
    else {
        if (!dirExists(base_dir_path)) {
            RCLCPP_ERROR_STREAM(logger, "The base directory indicated with the absolute path:\n" + base_dir_path + "\n does not exist! Adjust config/params.yaml file!");
            assert(false);
        }
    }
}


void IOUtils::writeStatsToFile(std::ofstream &out_stats_file,
                               Stats &stats,
                               int frame_cont,
                               uint32_t sec,
                               uint32_t nsec,
                               int64_t sim_duration) {
    out_stats_file << frame_cont     << " "
                   << sec            << " "
                   << nsec           << " "
                   << stats.accuracy << " " << stats.fake_accuracy  << " "
                   << stats.miou     << " " << stats.f1score        << " "
                   << stats.FPR      << " " << stats.TPR            << " "
                   << stats.FNR      << " " << stats.TNR            << " "
                   << sim_duration   << std::endl;
}

void IOUtils::loadNormalizationConfig(std::string &normalization_config_path,
                             int features_num,
                             std::vector<float> &min,
                             std::vector<float> &p2p) {

    std::ifstream input_normalization_config_file;
    input_normalization_config_file = std::ifstream(normalization_config_path);
    std::string line;
    size_t start, end;
    int col_cont;

    min.resize(features_num);
    p2p.resize(features_num);


    // load min
    start = 0, col_cont = 0;
    getline (input_normalization_config_file, line);
    assert(line.length()>0);
    for (size_t i=0; i<line.length(); i++) {
        assert(col_cont<features_num);
        if (line.at(i) == ' ' || line.at(i) == '\n' || i==line.length()-1) {
            end = i;
            min[col_cont] = (float) std::stod(line.substr(start, end));
            start = i;
            col_cont++;
        }
    }

    // load p2p
    start = 0, col_cont = 0;
    getline (input_normalization_config_file, line);
    assert(line.length()>0);
    for (size_t i=0; i<line.length(); i++) {
        assert(col_cont<features_num);
        if (line.at(i) == ' ' || line.at(i) == '\n' || i==line.length()-1) {
            end = i;
            p2p[col_cont] = (float) std::stod(line.substr(start, end));
            start = i;
            col_cont++;
        }
    }

    if (input_normalization_config_file.is_open())
        input_normalization_config_file.close();
}