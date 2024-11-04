#pragma once

#include "calibmar/extractors/charuco_board_extractor.h"
#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  class CharucoBoardTargetOptionsWidget : public QWidget {
   public:
    CharucoBoardTargetOptionsWidget(QWidget* parent = nullptr,bool is_system_calibration=false);

    void SetCharucoBoardTargetOptions(const CharucoBoardFeatureExtractor::Options& options);
    CharucoBoardFeatureExtractor::Options CharucoBoardTargetOptions();

   private:
    QComboBox* aruco_type_edit_;
    QSpinBox* rows_edit_;
    QSpinBox* columns_edit_;
    QSpinBox* marker_num_edit_;
    QSpinBox* board_index_edit_;
    QDoubleSpinBox* marker_size_edit_;
    QDoubleSpinBox* square_size_edit_;
    QLabel* label_;
    QSpinBox* frame_num_edit_;
  };
}