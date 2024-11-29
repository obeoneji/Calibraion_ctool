#include "calibration_system_options_widget.h"
#include "ui/utils/parse_params.h"
#include <colmap/util/misc.h>

namespace calibmar {
  CalibrationSystemOptionsWidget::CalibrationSystemOptionsWidget(QWidget* parent) : QWidget(parent) {
    // camera model
    camera_model_selector_ = new CameraSystemModelSelectorWidget(this);
    // calibration target groupbox
    calibration_target_options_ = new CalibrationTargetOptionsWidget(this,true);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(camera_model_selector_);
    // layout->addWidget(housing_type_selector_);
    layout->addWidget(calibration_target_options_);
    layout->setSizeConstraint(QLayout::SetMinimumSize);

    layout->setContentsMargins(0, 0, 0, 0);
  }

  bool CalibrationSystemOptionsWidget::Validate() {
    initial_camera_count_= camera_model_selector_->Initialcamera_counts();
    root_cam_=camera_model_selector_->Initialroot_cam();
    std::string message;
    if (!camera_model_selector_->Validate(message,initial_camera_count_.value())) {
      QMessageBox::information(this, "Validation Error", QString::fromStdString(message));
      return false;
    }
    // else {
    //   // validated
    //   initial_camera_parameters_ = camera_model_selector_->InitialCameraParameters(initial_camera_count_.value());
    // }
    if(!initial_camera_count_.value())
    {
      QMessageBox::information(this, "Validation Error", "camera count not valid");
      return false;
    }
    if(!root_cam_.value())
    {
      QMessageBox::information(this, "Validation Error", "root cam not valid");
      return false;
    }
    if(root_cam_.value()>initial_camera_count_.value())
    {
      QMessageBox::information(this, "Validation Error", 
          "root_cam: " + QString::number(root_cam_.value()) + 
          " > camera_count: " + QString::number(initial_camera_count_.value()));
      return false;
    }

    return true;
  }

  CalibrationSystemOptionsWidget::Options CalibrationSystemOptionsWidget::GetOptions() {
    Options options;
    options.calibration_target_options = calibration_target_options_->CalibrationTargetOptions();
    options.camera_model = camera_model_selector_->CameraModel();
    options.cam_num=initial_camera_count_;
    options.root_cam=root_cam_;
    return options;
  }

  void CalibrationSystemOptionsWidget::SetOptions(Options options) {
    camera_model_selector_->SetCameraModel(options.camera_model);
    camera_model_selector_->SetInitialCameranum(options.cam_num);
    camera_model_selector_->SetInitialrootcams(options.root_cam);
    calibration_target_options_->SetCalibrationTargetOptions(options.calibration_target_options);
  }

  void CalibrationSystemOptionsWidget::ForceArucoFor3DTarget(bool force) {
    calibration_target_options_->ForceArucoFor3DTarget(force);
  }
}