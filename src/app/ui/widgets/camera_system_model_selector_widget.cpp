#include "camera_system_model_selector_widget.h"
#include "ui/utils/parse_params.h"

#include <colmap/sensor/models.h>

namespace calibmar {

  CameraSystemModelSelectorWidget::CameraSystemModelSelectorWidget(QWidget* parent) : QGroupBox(parent) {
    setTitle("Camera model");

    camera_model_ = new CameraModelWidget(this);
    camera_system_inital_ = new CameraSystemInitalWidget(this);

 // 创建一个竖直布局
    QVBoxLayout* camera_model_layout = new QVBoxLayout(this);
    camera_model_layout->addWidget(camera_model_);

    camera_model_layout->addWidget(camera_system_inital_);

    setMinimumSize(100, 200); // 根据需要调整这个值
    // connect(camera_model_, &CameraModelWidget::onSpinBoxValueChanged, 
    //         initial_parameters_, &InitialParametersWidget::updateInitialParameters);


  }

  CameraModelType CameraSystemModelSelectorWidget::CameraModel() {
    return camera_model_->CameraModel();
  }

  void CameraSystemModelSelectorWidget::SetCameraModel(CameraModelType type) {
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

  std::vector<std::vector<double>> CameraSystemModelSelectorWidget::InitialCameraParameters(int camnum) {//liheng3
    std::vector<std::vector<double>> CameraParameters;
    for (int i=0;i<camnum;i++)
    {    
      std::optional<std::string> parameters_string = camera_system_inital_->InitialParameters(i);
      std::vector<double> params;
      if (!parameters_string.has_value() || !TryParseParams(params, parameters_string.value())) {
        params={};
      }
      CameraParameters.push_back(params);
    }
    return CameraParameters;
  }


  std::optional<int> CameraSystemModelSelectorWidget::Initialcamera_counts() {
    std::optional<int> count=camera_system_inital_->Initialcount();
    return count;
  }
  
  std::optional<int> CameraSystemModelSelectorWidget::Initialroot_cam() {
    std::optional<int> root=camera_system_inital_->Initialrootcam();
    return root;
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
  void CameraSystemModelSelectorWidget::SetInitialCameraParameters(const std::vector<std::vector<double>>& parameters) {//liheng3

    if (!parameters.empty()) {
      std::vector<std::string> csv_parameters;
      for (const auto& param_vector : parameters) {
        csv_parameters.push_back(colmap::VectorToCSV(param_vector)); // 转换每个向量为 CSV
        }
      camera_system_inital_->SetInitialParameters(csv_parameters);
      // initial_camera_count_->SetInitialcameranum();
    }
    else {
      camera_system_inital_->SetInitialParameters({});
    }

  }
void CameraSystemModelSelectorWidget::SetInitialCameranum(const std::optional<int>& count) {
    if (count.has_value())
    {
      camera_system_inital_->SetInitialcameranum(count);
    }
    else{
      camera_system_inital_->SetInitialcameranum({});
    }
  }

void CameraSystemModelSelectorWidget::SetInitialrootcams(const std::optional<int>& root) {
    if (root.has_value())
    {
      camera_system_inital_->SetInitialrootcam(root);
    }
    else{
      camera_system_inital_->SetInitialrootcam({});
    }
  }

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
  bool CameraSystemModelSelectorWidget::Validate(std::string& error_message,int cam_num) {
    CameraModelType camera_model = CameraModel();
    // if(cam_num!=2)
    // {
    // error_message = "camnum!=2.";
    //       return false;
    // }

    for (int i = 0; i < cam_num; ++i) {
      std::optional<std::string> parameters_string = camera_system_inital_->InitialParameters(i);
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