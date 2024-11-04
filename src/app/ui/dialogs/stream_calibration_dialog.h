#pragma once
#include "ui/widgets/common_calibration_options_widget.h"
#include "ui/widgets/image_widget.h"

#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  class StreamCalibrationDialog : public QDialog {
   public:
    enum class AcquisitionMode { OnButton, OnTimedDetection, OnSuggestedPose };

    struct Options {
      CalibrationTargetOptionsWidget::Options calibration_target_options = ChessboardFeatureExtractor::Options{4, 4, 1.0};
      CameraModelType camera_model = CameraModelType::SimplePinholeCameraModel;
      std::optional<std::pair<HousingInterfaceType, std::vector<double>>> housing_calibration = {};
      std::vector<std::vector<double>> initial_camera_parameters = {};//liheng3
      std::optional<std::string> save_images_directory = {};
      int device_index = 0;
      int cam_num_label = 0;
      // AcquisitionMode acquisition_mode = AcquisitionMode::OnTimedDetection;
    };

    StreamCalibrationDialog(QWidget* parent = nullptr);

    void SetOptions(Options options);
    Options GetOptions();

    virtual void done(int r) override;

   private:
    bool Validate();
    void ImportParameters();
    // void StartLiveStream(int index);
    // void CloseLiveStream();

    std::vector<std::pair<AcquisitionMode, std::string>> acquisition_modes_;
    std::unique_ptr<std::thread> worker_thread_;
    std::atomic<bool> cancel_;
    std::atomic<bool> valid_device_;

    std::optional<std::string> save_images_directory_;

    QComboBox* mode_combobox_;
    QSpinBox* device_index_;
    QSpinBox* cam_num_label_;
    CommonCalibrationOptionsWidget* calibration_options_widget_;
    ImageWidget* image_widget_;
    QLineEdit* directory_edit_;
    QCheckBox* save_images_checkbox_;
  };
}
