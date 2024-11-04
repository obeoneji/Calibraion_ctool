#pragma once

#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/core/report.h"
#include "ui/widgets/camera_model_widget.h"
#include "ui/widgets/camera_system_inital_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {
  class CameraSystemModelSelectorWidget : public QGroupBox {

  public:
    CameraSystemModelSelectorWidget(QWidget* parent = nullptr);

    CameraModelType CameraModel();
    void SetCameraModel(CameraModelType type);
    std::optional<int> Initialcamera_counts();
    std::optional<int> Initialroot_cam();
    // std::optional<std::vector<double>> InitialCameraParameters();
    std::vector<std::vector<double>> InitialCameraParameters(int camnum);

    // void SetInitialCameraParameters(const std::optional<std::vector<double>>& parameters);
    void SetInitialCameraParameters(const std::vector<std::vector<double>>& parameters);//liheng3

    void SetInitialCameranum(const std::optional<int>& count);
    void SetInitialrootcams(const std::optional<int>& root);
    bool Validate(std::string& error_message,int cam_num);//liheng3

  private:
    CameraModelWidget* camera_model_;
    CameraSystemInitalWidget* camera_system_inital_;
  };
}
