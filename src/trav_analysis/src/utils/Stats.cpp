
#include "Stats.h"


Stats::Stats() {
    resetStats();
}
void Stats::resetStats() {
    non_trav_num=0; trav_num=0;
    FP = 0;     FN = 0;     TP = 0;     TN = 0;     UNK = 0;
    FPR= .0f;   FNR= .0f;   TPR= .0f;   TNR= .0f;   UNKR = .0f;
    accuracy = .0f; fake_accuracy = .0f; miou = .0f; f1score = .0f;
}
void Stats::non_trav_increment() {non_trav_num++;}
void Stats::trav_increment() {trav_num++;}
void Stats::FP_increment() {FP++;}
void Stats::TP_increment() {TP++;}
void Stats::FN_increment() {FN++;}
void Stats::TN_increment() {TN++;}
void Stats::UNK_increment() {UNK++;}

void Stats::updateTrueLabelCont(float label) {
    if ( ISTRAVERSABLE(label) )
        trav_increment();
    else if ( ISNOTTRAVERSABLE(label) )
        non_trav_increment();

}
void Stats::updatePredLabelStats(float true_label, float pred_label) {
    if ( true_label==UNKNOWN_CELL_LABEL ) return;

    // il ground truth è noto ma la cella è predetta unknown. Verrà contata in tot_valid_points_in_gt
    if (pred_label == UNKNOWN_CELL_LABEL)
        UNK_increment();

    // abbiamo una label known per entrambi
    if (ISTRAVERSABLE(true_label)) {
        trav_increment();
        if ( ISTRAVERSABLE(pred_label) )
            TP_increment();
        else if ( ISNOTTRAVERSABLE(pred_label) )
            FN_increment();
    }
    else {
        non_trav_increment();
        if ( ISTRAVERSABLE(pred_label) )
            FP_increment();
        else if ( ISNOTTRAVERSABLE(pred_label) )
            TN_increment();
    }
}

void Stats::computeStats() {
    tot_predictions = non_trav_num + trav_num;

    if (tot_predictions == 0) { return; }
    non_trav_percentage = RATE(non_trav_num, tot_predictions);
    trav_percentage     = RATE(trav_num, tot_predictions);
    accuracy            = RATE(TN + TP, tot_predictions);

    int tot_local_predictions = TN + TP + FN + FP;
    if (tot_local_predictions == 0) fake_accuracy=.0f;
    else
        fake_accuracy   = RATE( TN+TP, tot_local_predictions );

    int tot_positive_labels = TP + FN + FP;
    if (tot_local_predictions == 0) miou=.0f;
    else
        miou = RATE( TP, tot_positive_labels );

    int armonic_mean_denom = ((TP << 1) + FP + FN);
    if (armonic_mean_denom == 0) f1score = .0f;
    else
        f1score = RATE( (TP << 1), armonic_mean_denom );

    int gt_true_negatives = (FP + TN);
    int gt_true_positives = (TP + FN);
    FPR = RATE(FP, gt_true_negatives);
    TPR = RATE(TP, gt_true_positives);
    FNR = RATE(FN, gt_true_positives); // = 1 - something
    TNR = RATE(TN, gt_true_negatives); // = 1 - something
    UNKR = RATE(UNK, (tot_predictions));
}

void Stats::printStats() {
    computeStats();

    int unique_setw = (int) (std::to_string(tot_predictions).length());
    int perc_setw = 0, temp_setw;
    float perc[] = {non_trav_percentage, trav_percentage, accuracy, FPR, TPR, FNR, TNR, fake_accuracy, miou, f1score};
    for (int i=0; i<7; i++) if ((temp_setw = (int) (std::to_string(perc[i]).length())) > perc_setw) perc_setw = temp_setw;

    ROS_INFO_STREAM("   -- unique non_trav: "                                       << std::setw(perc_setw) << TO_PERC(non_trav_percentage) << " % [" << std::setw(unique_setw) <<  non_trav_num << " cells]");
    ROS_INFO_STREAM(std::setprecision(3) << "   -- unique trav    : "           << std::setw(perc_setw) << TO_PERC(trav_percentage)     << " % [" << std::setw(unique_setw) <<  trav_num << " cells]");
    ROS_INFO_STREAM(std::setprecision(3) << "   -- accuracy       : \033[1;32m" << std::setw(perc_setw) << TO_PERC(accuracy)            << "\033[0m %");
    ROS_INFO_STREAM(std::setprecision(3) << "   -- local accuracy : \033[1;32m" << std::setw(perc_setw) << TO_PERC(fake_accuracy)            << "\033[0m %");
    ROS_INFO_STREAM(std::setprecision(3) << "   -- miou           : \033[1;32m" << std::setw(perc_setw) << TO_PERC(miou)            << "\033[0m %");
    ROS_INFO_STREAM(std::setprecision(3) << "   -- f1score        : \033[1;32m" << std::setw(perc_setw) << TO_PERC(f1score)            << "\033[0m %");
    ROS_INFO_STREAM(std::setprecision(3) << "   -- FP rate        : "           << std::setw(perc_setw) << TO_PERC(FPR)                 << " % [" << std::setw(unique_setw) << FP << " / " << non_trav_num << " cells ]");
    ROS_INFO_STREAM(std::setprecision(3) << "   -- TP rate        : "           << std::setw(perc_setw) << TO_PERC(TPR)                 << " % [" << std::setw(unique_setw) << TP << " / " << trav_num << " cells ]");
    ROS_INFO_STREAM(std::setprecision(3) << "   -- FN rate        : "               << std::setw(perc_setw) << TO_PERC(FNR)                 << " % [" << std::setw(unique_setw) << FN << " / " << trav_num << " cells ]");
    ROS_INFO_STREAM(std::setprecision(3) << "   -- TN rate        : "           << std::setw(perc_setw) << TO_PERC(TNR)                 << " % [" << std::setw(unique_setw) << TN << " / " << non_trav_num << " cells ]");
    ROS_INFO_STREAM(std::setprecision(3) << "   -- UNK rate       : "           << std::setw(perc_setw) << TO_PERC(UNKR)                << " % [" << std::setw(unique_setw) << UNK<< " / " << tot_predictions << " cells ]");
}

void Stats::printStats(std::string title) {
    ROS_INFO_STREAM("");
    ROS_WARN_STREAM(title);
    printStats();
}