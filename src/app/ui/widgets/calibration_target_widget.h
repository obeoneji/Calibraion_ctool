#pragma once

#include <QtCore>
#include <QtWidgets>
#include <optional>

#include "aruco_board_target_widget.h"
#include "chessboard_target_widget.h"
#include "target3D_target_widget.h"
#include "charuco_board_target_widget.h"

namespace calibmar {

  class CalibrationTargetOptionsWidget : public QGroupBox {
   public:
    typedef std::variant<ChessboardFeatureExtractor::Options, ArucoBoardFeatureExtractor::Options,
                         ArucoSiftFeatureExtractor::Options, SiftFeatureExtractor::Options,CharucoBoardFeatureExtractor::Options>
        Options;

    CalibrationTargetOptionsWidget(QWidget* parent = nullptr,bool is_system_calibration=false);

    void SetCalibrationTargetOptions(const Options& options);
    Options CalibrationTargetOptions();

    void ForceArucoFor3DTarget(bool force);

   private:
    void SetCalibrationTargetType(int index);
    std::vector<QWidget*> widgets;
    QComboBox* target_type_combobox_;
    Target3DTargetOptionsWidget* target3D_target_widget_;
    ChessboardTargetOptionsWidget* chessboard_target_widget_;
    ArucoBoardTargetOptionsWidget* aruco_board_target_widget_;
    CharucoBoardTargetOptionsWidget* charuco_board_target_widget_;
  };
}