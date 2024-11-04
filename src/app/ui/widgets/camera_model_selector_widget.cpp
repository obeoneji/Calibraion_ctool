#include "camera_model_selector_widget.h"
#include "ui/utils/parse_params.h"

#include <colmap/sensor/models.h>

namespace calibmar {

  CameraModelSelectorWidget::CameraModelSelectorWidget(QWidget* parent) : QGroupBox(parent) {
    setTitle("Camera model");

    camera_model_ = new CameraModelWidget(this);
    initial_parameters_ = new InitialParametersWidget(this);

 // 创建一个竖直布局
    QVBoxLayout* camera_model_layout = new QVBoxLayout(this);
    camera_model_layout->addWidget(camera_model_);

    camera_model_layout->addWidget(initial_parameters_);

    setMinimumSize(300, 300); // 根据需要调整这个值
    // connect(camera_model_, &CameraModelWidget::onSpinBoxValueChanged, 
    //         initial_parameters_, &InitialParametersWidget::updateInitialParameters);


  }

  CameraModelType CameraModelSelectorWidget::CameraModel() {
    return camera_model_->CameraModel();
  }

  void CameraModelSelectorWidget::SetCameraModel(CameraModelType type) {
    camera_model_->SetCameraModel(type);
  }

  // std::optional<std::vector<double>> CameraModelSelectorWidget::InitialCameraParameters() {
  //   std::optional<std::string> parameters_string = initial_parameters_->InitialParameters();
  //   std::vector<double> params;
  //   if (!parameters_string.has_value() || !TryParseParams(params, parameters_string.value())) {
  //     return {};
  //   }

  //   return params;
  // }

  std::vector<std::vector<double>> CameraModelSelectorWidget::InitialCameraParameters(int camnum) {//liheng3
    std::vector<std::vector<double>> CameraParameters;
    for (int i=0;i<camnum;i++)
    {    
      std::optional<std::string> parameters_string = initial_parameters_->InitialParameters(i);
      std::vector<double> params;
      if (!parameters_string.has_value() || !TryParseParams(params, parameters_string.value())) {
        params={};
      }
      CameraParameters.push_back(params);
    }
    return CameraParameters;
  }


  std::optional<int> CameraModelSelectorWidget::Initialcamera_counts() {
    std::optional<int> count=initial_parameters_->Initialcount();
    return count;
  }

  // void CameraModelSelectorWidget::SetInitialCameraParameters(const std::optional<std::vector<double>>& parameters) {
  //   if (parameters.has_value()) {
  //     initial_parameters_->SetInitialParameters(colmap::VectorToCSV(parameters.value()));
  //     // initial_camera_count_->SetInitialcameranum();
  //   }
  //   else {
  //     initial_parameters_->SetInitialParameters({});
  //   }

  // }
  void CameraModelSelectorWidget::SetInitialCameraParameters(const std::vector<std::vector<double>>& parameters) {//liheng3

    if (!parameters.empty()) {
      std::vector<std::string> csv_parameters;
      for (const auto& param_vector : parameters) {
        csv_parameters.push_back(colmap::VectorToCSV(param_vector)); // 转换每个向量为 CSV
        }
      initial_parameters_->SetInitialParameters(csv_parameters);
      // initial_camera_count_->SetInitialcameranum();
    }
    else {
      initial_parameters_->SetInitialParameters({});
    }

  }
void CameraModelSelectorWidget::SetInitialCameranum(const std::optional<int>& count) {
    if (count.has_value())
    {
      initial_parameters_->SetInitialcameranum(count);
    }
    else{
      initial_parameters_->SetInitialcameranum({});
    }
  }
// void CameraModelSelectorWidget::SetInitialrootcams(const std::optional<int>& root) {
//     if (root.has_value())
//     {
//       initial_parameters_->SetInitialrootcam(root);
//     }
//     else{
//       initial_parameters_->SetInitialrootcam({});
//     }
//   }


  // bool CameraModelSelectorWidget::Validate(std::string& error_message,int cam_num) {
  //   CameraModelType camera_model = CameraModel();
  //   std::optional<std::string> parameters_string = initial_parameters_->InitialParameters();
  //   if (parameters_string.has_value()) {
  //     std::vector<double> params;
  //     if (!TryParseParams(params, parameters_string.value())) {
  //       error_message = "Invalid camera parameter format.";
  //       return false;
  //     }
  //     else if (params.size() != CameraModel::CameraModels().at(camera_model).num_params) {
  //       error_message = "Camera parameters dont match camera model.";
  //       return false;
  //     }
  //   }

  //   return true;
  // }
  bool CameraModelSelectorWidget::Validate(std::string& error_message,int cam_num) {
    CameraModelType camera_model = CameraModel();
    // if(cam_num!=2)
    // {
    // error_message = "camnum!=2.";
    //       return false;
    // }

    for (int i = 0; i < cam_num; ++i) {
      std::optional<std::string> parameters_string = initial_parameters_->InitialParameters(i);
      if (parameters_string.has_value()) {
        std::vector<double> params;
        if (!TryParseParams(params, parameters_string.value())) {
          error_message = "Invalid camera parameter format.";
          return false;
        }
        else if (params.size() != CameraModel::CameraModels().at(camera_model).num_params) {
          error_message = "Camera parameters dont match camera model.";
          return false;
        }
      }
    }
    return true;
  }


}