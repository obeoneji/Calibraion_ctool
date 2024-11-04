#pragma once

#include "ui/widgets/common_calibration_options_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class FileCalibrationDialog : public QDialog {
   public:
    struct Options {
      CalibrationTargetOptionsWidget::Options calibration_target_options;
      CameraModelType camera_model;
      std::optional<std::pair<HousingInterfaceType, std::vector<double>>> housing_calibration;
      // std::optional<std::vector<double>> initial_camera_parameters;
      std::vector<std::vector<double>> initial_camera_parameters;//liheng3
      std::string images_directory;
      std::optional<int> cam_num;
      int current_index;
    };

    FileCalibrationDialog(QWidget* parent = nullptr);

    void SetOptions(Options options);
    Options GetOptions();

   private:
    bool Validate();
    void ImportParameters();

    QLineEdit* directory_edit_;
    CommonCalibrationOptionsWidget* calibration_options_widget_;
  };
}
