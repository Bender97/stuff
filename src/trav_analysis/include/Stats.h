
#pragma once

#ifndef TRAV_ANALYSIS_STATS_H
#define TRAV_ANALYSIS_STATS_H

#include "utility.h"

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
class Stats {
public:
    int non_trav_num, trav_num;
    int tot_predictions;
    int     FP,  FN,  TP,  TN, UNK;
    float   FPR, FNR, TPR, TNR, UNKR; // * rates
    float non_trav_percentage, trav_percentage;
    float accuracy, fake_accuracy, miou, f1score;


    Stats();
    void resetStats();
    void non_trav_increment();
    void trav_increment();
    void FP_increment();
    void TP_increment();
    void FN_increment();
    void TN_increment();
    void UNK_increment();

    void updateTrueLabelCont(float label);
    void updatePredLabelStats(float true_label, float pred_label);

    void computeStats();

    void printStats(rclcpp::Logger &logger);

    void printStats(std::string title, rclcpp::Logger &logger);
};

#endif // TRAV_ANALYSIS_STATS_H