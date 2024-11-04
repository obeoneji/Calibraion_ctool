#pragma once

#include "calibmar/core/camera_models.h"

#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  class CameraModelWidget : public QWidget {
    // Q_OBJECT
  public:
    CameraModelWidget(QWidget* parent = nullptr);
    CameraModelWidget(const std::function<void()> model_changed_callback, QWidget* parent = nullptr);

    CameraModelType CameraModel();
    void SetCameraModel(CameraModelType type);

    void SetCameraParametersLabel(int index);
  // signals:
  //   void onSpinBoxValueChanged(int value);

  private:
    std::vector<std::tuple<CameraModelType, std::string, std::string>> camera_models_;
    QComboBox* camera_model_combobox_;
    QLabel* camera_parameters_label_;
    QLabel* cam_label;
    // QSpinBox* cam_label_edit_;
    std::function<void()> model_changed_callback_;
  };
}
