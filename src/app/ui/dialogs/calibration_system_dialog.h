#pragma once

#include "ui/widgets/calibration_system_options_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class CalibrationSystemDialog : public QDialog {
   public:
    struct Options {
      CalibrationTargetOptionsWidget::Options calibration_target_options;
      CameraModelType camera_model;
      // std::optional<std::vector<double>> initial_camera_parameters;
      // std::vector<std::vector<double>> initial_camera_parameters;//liheng3
      std::string images_directory;
      std::string output_images_directory; 
      bool g2o_options;
      std::optional<int> cam_num;
      std::optional<int> root_cam;
      int current_index;
      int solver_index;
    };

    CalibrationSystemDialog(QWidget* parent = nullptr);

    void SetOptions(Options options);
    Options GetOptions();

   private:
    bool Validate();
    void ImportParameters();

    QLineEdit* directory_edit_;
    QLineEdit* output_directory_edit_;
    QComboBox* g2o_solver_combobox_ ;
    QCheckBox* g2o_checkbox_ ;
    QLabel* g2o_solver_label;
    CalibrationSystemOptionsWidget* calibration_system_options_widget_;
  };
}
