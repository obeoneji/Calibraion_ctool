#include "files_calibration_runner.h"
#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/calibrators/calibrator3D.h"
#include "calibmar/calibrators/housing_calibrator.h"
#include "calibmar/extractors/aruco_sift_extractor.h"
#include "calibmar/extractors/sift_extractor.h"
#include "calibmar/extractors/charuco_board_extractor.h"
#include "calibmar/readers/filesystem_reader.h"

#include "ui/widgets/calibration_result_widget.h"

#include <colmap/scene/reconstruction.h>

namespace {
  void SetupPlanarCalibration(
      std::variant<calibmar::ChessboardFeatureExtractor::Options,calibmar::CharucoBoardFeatureExtractor::Options,calibmar::ArucoBoardFeatureExtractor::Options>& target_options,
      const calibmar::FileCalibrationDialog::Options& options, calibmar::Calibration& calibration, std::pair<int, int> image_size,
      std::unique_ptr<calibmar::FeatureExtractor>& extractor, std::unique_ptr<calibmar::TargetVisualizer>& target_visualizer,
      std::unique_ptr<calibmar::Calibrator>& calibrator) {
    using namespace calibmar;

    ChessboardFeatureExtractor::Options* chessboard_options =
        std::get_if<calibmar::ChessboardFeatureExtractor::Options>(&target_options);
    ArucoBoardFeatureExtractor::Options* aruco_options =
        std::get_if<calibmar::ArucoBoardFeatureExtractor::Options>(&target_options);
    CharucoBoardFeatureExtractor::Options* charucoboard_options = std::get_if<calibmar::CharucoBoardFeatureExtractor::Options>(&target_options);

    if (chessboard_options) {
      chessboard_options->fast = false;
      std::unique_ptr<ChessboardFeatureExtractor> chessboard_extractor =
          std::make_unique<ChessboardFeatureExtractor>(*chessboard_options);
      calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(*chessboard_options));
      calibration.SetPoints3D(chessboard_extractor->Points3D());
      extractor = std::move(chessboard_extractor);
      target_visualizer = std::make_unique<ChessboardTargetVisualizer>(chessboard_options->chessboard_columns,
                                                                       chessboard_options->chessboard_rows);
    }
    else if(charucoboard_options){
      CharucoBoardFeatureExtractor::Options& charuco_options = std::get<calibmar::CharucoBoardFeatureExtractor::Options>(target_options);
      std::unique_ptr<CharucoBoardFeatureExtractor> chaurco_extractor = std::make_unique<CharucoBoardFeatureExtractor>(charuco_options);
      calibration.SetPoints3D(chaurco_extractor->Points3D());
      calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(charuco_options));
      extractor = std::move(chaurco_extractor);
      target_visualizer = std::make_unique<CharucoboardTargetVisualizer>(charuco_options.columns,charuco_options.rows);
    }
    else{
      ArucoBoardFeatureExtractor::Options& aruco_options = std::get<calibmar::ArucoBoardFeatureExtractor::Options>(target_options);
      std::unique_ptr<ArucoBoardFeatureExtractor> aurco_extractor = std::make_unique<ArucoBoardFeatureExtractor>(aruco_options);
      calibration.SetPoints3D(aurco_extractor->Points3D());
      calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(aruco_options));
      extractor = std::move(aurco_extractor);
      target_visualizer = std::make_unique<ArucoBoardTargetVisualizer>();
    }


    if (options.housing_calibration.has_value()) {
      HousingCalibrator::Options calibrator_options;
      calibrator_options.camera_model = options.camera_model;
      calibrator_options.image_size = image_size;
      calibrator_options.current_cam_index =options.current_index ;
      calibrator_options.estimate_initial_dome_offset =
          (charucoboard_options) && options.housing_calibration.value().first == HousingInterfaceType::DoubleLayerSphericalRefractive;
      calibrator_options.housing_interface = options.housing_calibration.value().first;
      // calibrator_options.camera_params = options.initial_camera_parameters.value();
      calibrator_options.camera_params = options.initial_camera_parameters;//liheng3
      calibrator_options.initial_housing_params = options.housing_calibration.value().second;
      if (chessboard_options) {
        calibrator_options.pattern_cols_rows = {chessboard_options->chessboard_columns, chessboard_options->chessboard_rows};
      }
      else if (charucoboard_options)
      {
        calibrator_options.pattern_cols_rows = {charucoboard_options->columns, charucoboard_options->rows};
      }
      calibrator = std::make_unique<HousingCalibrator>(calibrator_options);
    }
    else {
      BasicCalibrator::Options calibrator_options;
      // if (options.initial_camera_parameters.has_value()) {   //liheng3
      if (!options.initial_camera_parameters.empty()) {
        calibration.SetCamera(
            // CameraModel::InitCamera(options.camera_model, image_size, options.initial_camera_parameters.value()));
            CameraModel::InitCamera(options.camera_model, image_size, options.initial_camera_parameters[0]));//liheng3
        calibrator_options.use_intrinsics_guess = true;
      }
      else {
        calibrator_options.camera_model = options.camera_model;
        calibrator_options.image_size = image_size;
      }
      calibrator = std::make_unique<BasicCalibrator>(calibrator_options);
    }
  }

  void Setup3DTargetCalibration(calibmar::Target3DTargetOptionsWidget::Options target3d_options,
                                const calibmar::FileCalibrationDialog::Options& options, calibmar::Calibration& calibration,
                                std::pair<int, int> image_size, std::unique_ptr<calibmar::FeatureExtractor>& extractor,
                                std::unique_ptr<calibmar::TargetVisualizer>& target_visualizer,
                                std::unique_ptr<calibmar::Calibrator>& calibrator) {
    using namespace calibmar;

    calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(target3d_options));
    Calibrator3D::Options calibrator_options;
    bool use_aruco = std::holds_alternative<ArucoSiftFeatureExtractor::Options>(target3d_options);

    calibrator_options.contains_arucos = use_aruco;
    // if (options.initial_camera_parameters.has_value()) {
    if (!options.initial_camera_parameters.empty()) {
      // calibration.SetCamera(CameraModel::InitCamera(options.camera_model, image_size, options.initial_camera_parameters.value()));
      calibration.SetCamera(CameraModel::InitCamera(options.camera_model, image_size, options.initial_camera_parameters[0]));
      calibrator_options.use_intrinsics_guess = true;
    }
    else {
      calibrator_options.camera_model = options.camera_model;
      calibrator_options.image_size = image_size;
    }

    if (options.housing_calibration.has_value()) {
      calibration.Camera().refrac_model_id = colmap::CameraRefracModelNameToId(
          calibmar::HousingInterface::HousingInterfaces().at(options.housing_calibration.value().first).model_name);
      calibration.Camera().refrac_params = options.housing_calibration.value().second;
      calibrator_options.enable_refraction = true;
    }

    double aruco_mask_factor = 1;
    if (use_aruco) {
      ArucoSiftFeatureExtractor::Options options = std::get<ArucoSiftFeatureExtractor::Options>(target3d_options);
      aruco_mask_factor = options.masking_scale_factor;
      extractor = std::make_unique<ArucoSiftFeatureExtractor>(options);
    }
    else {
      SiftFeatureExtractor::Options options = std::get<SiftFeatureExtractor::Options>(target3d_options);
      extractor = std::make_unique<SiftFeatureExtractor>(options);
    }

    calibrator = std::make_unique<Calibrator3D>(calibrator_options);
    target_visualizer = std::make_unique<Target3DTargetVisualizer>(use_aruco, aruco_mask_factor);
  }
}

namespace calibmar {

  FilesCalibrationRunner::FilesCalibrationRunner(CalibrationWidget* calibration_widget, FileCalibrationDialog::Options options)
      : calibration_widget_(calibration_widget), options_(options) {}

  bool FilesCalibrationRunner::Run(Calibration& calibration) {
    FilesystemImageReader::Options reader_options;
    reader_options.image_directory = options_.images_directory;
    int cam_count=options_.cam_num.value();
    calibration.addcamnum(cam_count);

    // reader_options.cam_index = cam_count;
    reader_options.cam_index =options_.current_index ;//liheng1
    FilesystemImageReader reader(reader_options);
    std::pair<int, int> image_size;
    try {
      image_size = {reader.ImagesWidth(), reader.ImagesHeight()};
    }
    catch (std::exception& ex) {
      QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_]() {
        calibration_widget->EndCalibration(new CalibrationResultWidget("No readable images in provided directory!"));
      });
      return false;
    }

    std::unique_ptr<FeatureExtractor> extractor;
    std::unique_ptr<TargetVisualizer> target_visualizer;
    std::unique_ptr<Calibrator> calibrator;

    bool is_3D_calibration = std::holds_alternative<ArucoSiftFeatureExtractor::Options>(options_.calibration_target_options) ||
                            std::holds_alternative<SiftFeatureExtractor::Options>(options_.calibration_target_options);

    if (is_3D_calibration) {
      Target3DTargetOptionsWidget::Options target3d_options;
      if (std::holds_alternative<ArucoSiftFeatureExtractor::Options>(options_.calibration_target_options)) {
        target3d_options = std::get<ArucoSiftFeatureExtractor::Options>(options_.calibration_target_options);
      }
      else {
        target3d_options = std::get<SiftFeatureExtractor::Options>(options_.calibration_target_options);
      }

      Setup3DTargetCalibration(target3d_options, options_, calibration, image_size, extractor, target_visualizer, calibrator);
    }
    else {
      std::variant<calibmar::ChessboardFeatureExtractor::Options,calibmar::CharucoBoardFeatureExtractor::Options,calibmar::ArucoBoardFeatureExtractor::Options> target_options;
      if (std::holds_alternative<ChessboardFeatureExtractor::Options>(options_.calibration_target_options)) {
        target_options = std::get<ChessboardFeatureExtractor::Options>(options_.calibration_target_options);
      }
      else if(std::holds_alternative<ArucoBoardFeatureExtractor::Options>(options_.calibration_target_options)) {
        target_options = std::get<ArucoBoardFeatureExtractor::Options>(options_.calibration_target_options);
      }
      else
      {
        target_options = std::get<CharucoBoardFeatureExtractor::Options>(options_.calibration_target_options);
      }
      SetupPlanarCalibration(target_options, options_, calibration, image_size, extractor, target_visualizer, calibrator);
    }

    calibration_widget_->SetTargetVisualizer(std::move(target_visualizer));
    int image_detected=0;
    try {
      while (reader.HasNext()) {
        Image image;
        std::unique_ptr<Pixmap> pixmap = std::make_unique<Pixmap>();
        ImageReader::Status reader_status = reader.Next(image, *pixmap);
        FeatureExtractor::Status extractor_status;
        std::unique_ptr<ExtractionImageWidget::Data> data = std::make_unique<ExtractionImageWidget::Data>();
        if (reader_status == ImageReader::Status::SUCCESS) {
          data->image_name = image.Name();

          extractor_status = extractor->Extract(image, *pixmap);

          if (extractor_status == FeatureExtractor::Status::SUCCESS) {
            size_t id = calibration.AddImage(image);
            data->image_data = calibration.Image(id);
            image_detected+=1;
          }
          // else if(extractor_status == FeatureExtractor::Status::LACK_ERROR)  {
          //   throw std::runtime_error("LACK_ERROR .");
          // }
          // else{
          //   throw std::runtime_error("DETECTED ERROR .");
          // }
          else{
            continue;
          }
          // save a copy of the last image for the offset visualization
          last_pixmap_ = std::make_unique<Pixmap>(pixmap->Clone());
          data->image = std::move(pixmap);
          data->status = ExtractionImageWidget::ConvertStatus(extractor_status);
        }
        else {
          data->status = ExtractionImageWidget::ConvertStatus(reader_status);
        }

        // currently ignore read errors. Check visualization, when reenabling.
        if (data->status == ExtractionImageWidget::Status::SUCCESS ||
            data->status == ExtractionImageWidget::Status::DETECTION_ERROR ||
            data->status == ExtractionImageWidget::Status::IMAGE_DIMENSION_MISSMATCH) {
          QMetaObject::invokeMethod(calibration_widget_,
                                    [calibration_widget = calibration_widget_, data = std::move(data)]() mutable {
            calibration_widget->AddExtractionItem(
                new ExtractionImageWidget(std::move(data), calibration_widget->TargetVisualizer()));
          });
        }
      }

      QMetaObject::invokeMethod(calibration_widget_,
                                [calibration_widget = calibration_widget_]() { calibration_widget->StartCalibration(); });
      if(image_detected<=0)
      {
        throw std::runtime_error("Need enough data to calibrate dome port offset.");
      }
      calibrator->Calibrate(calibration);
    }
    catch (std::exception& ex) {
      std::string message(ex.what());

      QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_, message]() {
        calibration_widget->EndCalibration(new CalibrationResultWidget(message));
      });
      return false;
    }
    std::shared_ptr<colmap::Reconstruction> reconstruction;
    if (is_3D_calibration) {
      reconstruction = static_cast<Calibrator3D*>(calibrator.get())->Reconstruction();
    }

    QMetaObject::invokeMethod(calibration_widget_,
                              [calibration_widget = calibration_widget_, last_pixmap = std::move(last_pixmap_), &calibration,
                                reconstruction]() mutable {
      calibration_widget->EndCalibration(new CalibrationResultWidget(calibration, std::move(last_pixmap), reconstruction));
    });

  
    return true;
  }
}
