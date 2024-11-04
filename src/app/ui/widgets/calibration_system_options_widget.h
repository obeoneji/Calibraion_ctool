#pragma once

#include "calibmar/core/report.h"
#include "ui/widgets/calibration_target_widget.h"
#include "ui/widgets/camera_system_model_selector_widget.h"


#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {
  class CalibrationSystemOptionsWidget : public QWidget {
   public:
    struct Options {
      CameraModelType camera_model;
      std::vector<std::vector<double>> initial_camera_parameters;
      CalibrationTargetOptionsWidget::Options calibration_target_options;
      std::optional<int> cam_num;
      std::optional<int> root_cam;
      int solver_index;
    };

    CalibrationSystemOptionsWidget(QWidget* parent = nullptr);

    bool Validate();

    Options GetOptions();
    void SetOptions(Options options);

    void ForceArucoFor3DTarget(bool force);

   private:
    CameraSystemModelSelectorWidget* camera_model_selector_;
    // HousingSelectorWidget* housing_type_selector_;
    CalibrationTargetOptionsWidget* calibration_target_options_;
    std::vector<std::vector<double>> initial_camera_parameters_;
    std::optional<int> initial_camera_count_; // 存储相机数量
    std::optional<int> root_cam_; // 存储root相机
  };
}