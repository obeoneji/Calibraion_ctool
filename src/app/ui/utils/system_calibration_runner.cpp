#include "system_calibration_runner.h"
#include "calibmar/core/undistort.h"
#include <filesystem>
#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/calibrators/calibrator3D.h"
#include "calibmar/calibrators/housing_calibrator.h"
#include "calibmar/extractors/aruco_sift_extractor.h"
#include "calibmar/extractors/sift_extractor.h"
#include "calibmar/extractors/charuco_board_extractor.h"
#include "calibmar/extractors/multi_charuco_board_extractor.h"
#include "calibmar/readers/filesystem_reader.h"

#include "ui/widgets/calibration_result_widget.h"

#include <colmap/scene/reconstruction.h>

namespace {
  void SetupPlanarCalibration(
      std::variant<calibmar::ChessboardFeatureExtractor::Options,calibmar::CharucoBoardFeatureExtractor::Options,calibmar::ArucoBoardFeatureExtractor::Options>& target_options,
      const calibmar::CalibrationSystemDialog::Options& options, calibmar::Calibration& calibration, std::pair<int, int> image_size,
      std::unique_ptr<calibmar::FeatureExtractor>& extractor, std::unique_ptr<calibmar::TargetVisualizer>& target_visualizer,
      std::unique_ptr<calibmar::Calibrator>& calibrator) {
    using namespace calibmar;

    ChessboardFeatureExtractor::Options* chessboard_options =
        std::get_if<calibmar::ChessboardFeatureExtractor::Options>(&target_options);
    ArucoBoardFeatureExtractor::Options* aruco_options = std::get_if<calibmar::ArucoBoardFeatureExtractor::Options>(&target_options);
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
      std::unique_ptr<CharucoBoardFeatureExtractor> charuco_extractor =std::make_unique<CharucoBoardFeatureExtractor>(*charucoboard_options);
      // CharucoBoardFeatureExtractor::Options& charuco_options = std::get<calibmar::CharucoBoardFeatureExtractor::Options>(target_options);
      // std::unique_ptr<CharucoBoardFeatureExtractor> chaurco_extractor = std::make_unique<CharucoBoardFeatureExtractor>(charuco_options);
      calibration.SetPoints3D(charuco_extractor->Points3D());
      calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(*charucoboard_options));
      extractor = std::move(charuco_extractor);
      target_visualizer = std::make_unique<CharucoboardTargetVisualizer>(charucoboard_options->columns,charucoboard_options->rows);
    }
    else{
      ArucoBoardFeatureExtractor::Options& aruco_options = std::get<calibmar::ArucoBoardFeatureExtractor::Options>(target_options);
      std::unique_ptr<ArucoBoardFeatureExtractor> aurco_extractor = std::make_unique<ArucoBoardFeatureExtractor>(aruco_options);
      calibration.SetPoints3D(aurco_extractor->Points3D());
      calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(aruco_options));
      extractor = std::move(aurco_extractor);
      target_visualizer = std::make_unique<ArucoBoardTargetVisualizer>();
    }



    BasicCalibrator::Options calibrator_options;
    // if (!options.initial_camera_parameters.empty()) {
    //   calibration.SetCamera(
    //       // CameraModel::InitCamera(options.camera_model, image_size, options.initial_camera_parameters.value()));
    //       CameraModel::InitCamera(options.camera_model, image_size, options.initial_camera_parameters[0]));//liheng3
    //   calibrator_options.use_intrinsics_guess = true;
    // }
    // else {
    calibrator_options.camera_model = options.camera_model;
    calibrator_options.image_size = image_size;
    calibrator_options.cam_num =options.cam_num.value();
    calibrator_options.root_cam=options.root_cam.value();
    calibrator_options.solver_index=options.solver_index;
    calibrator_options.frame_num =charucoboard_options->frame_num;
    calibrator_options.marker_num=charucoboard_options->marker_num; 
    calibrator_options.board_num=charucoboard_options->board_index;
    calibrator_options.pattern=(charucoboard_options->columns-1)*(charucoboard_options->rows-1);
    // }
    calibrator = std::make_unique<BasicCalibrator>(calibrator_options);
    
  }
  void UndistortImages(calibmar::FilesystemImageReader& reader, colmap::Camera& distorted_camera, std::filesystem::path output_dir) {
    using namespace calibmar;
    calibmar::Image image;
    while (reader.HasNext()) {
      std::unique_ptr<Pixmap> distorted_pixmap = std::make_unique<Pixmap>();
      Pixmap undistorted_pixmap;

      if (reader.Next(image, *distorted_pixmap) != FilesystemImageReader::Status::SUCCESS) {
        continue;
      }
      colmap::Camera undistorted_camera;
      undistort::UndistortImage(std::move(distorted_pixmap), distorted_camera, undistorted_pixmap, undistorted_camera);

      std::filesystem::path file_name(image.Name());
      file_name = file_name.stem() += file_name.extension();
      std::filesystem::path output_file = output_dir / file_name;
      undistorted_pixmap.Write(output_file.string());
    }
  }
}

namespace calibmar {

  SystemCalibrationRunner::SystemCalibrationRunner(CalibrationWidget* calibration_widget, CalibrationSystemDialog::Options options)
      : calibration_widget_(calibration_widget), options_(options) {}

  bool SystemCalibrationRunner::Run(Calibration& calibration) {
    std::unique_ptr<FeatureExtractor> extractor;
    std::unique_ptr<TargetVisualizer> target_visualizer;
    std::unique_ptr<Calibrator> calibrator;


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
    // throw std::runtime_error("Error occurred with value: " + std::to_string(cam_count));
    int cam_count=options_.cam_num.value();   
    calibration.addcamnum(cam_count);
    calibration.issystem();
    calibration.isg2o(options_.g2o_options,options_.solver_index);
    FilesystemImageReader::Options reader_options;
    reader_options.image_directory = options_.images_directory;
    reader_options.cam_index = 0;//liheng1
    FilesystemImageReader reader1(reader_options);
    std::pair<int, int> image_size;
    try {
      image_size = {reader1.ImagesWidth(), reader1.ImagesHeight()};
    }
    catch (std::exception& ex) {
      QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_]() {
        calibration_widget->EndCalibration(new CalibrationResultWidget("No readable images in provided directory!"));
      });
      return false;
    }    

    SetupPlanarCalibration(target_options, options_, calibration, image_size, extractor, target_visualizer, calibrator);
    calibration_widget_->SetTargetVisualizer(std::move(target_visualizer));

    for(int i=0;i<cam_count;i++)
    {
      FilesystemImageReader::Options reader_options;
      reader_options.image_directory = options_.images_directory;
      reader_options.image_read_mode = Pixmap::ReadMode::COLOR_AS_SOURCE;
      // reader_options.cam_index = 0;//liheng1
      reader_options.cam_index = i;//liheng1
      FilesystemImageReader reader(reader_options);    
      try {
        while (reader.HasNext()) {
          Image image;
          image.issystem();
          std::unique_ptr<Pixmap> pixmap = std::make_unique<Pixmap>();
          ImageReader::Status reader_status = reader.Next(image, *pixmap);
          FeatureExtractor::Status extractor_status;
          std::unique_ptr<ExtractionImageWidget::Data> data = std::make_unique<ExtractionImageWidget::Data>();
          if (reader_status == ImageReader::Status::SUCCESS) 
          {
            data->image_name = image.Name();
            data->system=true;
            extractor_status = extractor->Extract(image, *pixmap);
            if (extractor_status == FeatureExtractor::Status::SUCCESS) 
            { 
              
            size_t id = calibration.AddImage_cam(image,i);
            data->image_data = calibration.Image_cam(id,i);
            }
            // else if(extractor_status == FeatureExtractor::Status::LACK_ERROR)  {
            //   throw std::runtime_error("LACK_ERROR .");
            // }
            // else{
            //   throw std::runtime_error("DETECTED ERROR .");
            // }


            // save a copy of the last image for the offset visualization
            last_pixmap_ = std::make_unique<Pixmap>(pixmap->Clone());
            data->image = std::move(pixmap);
            data->status = ExtractionImageWidget::ConvertStatus(extractor_status);
          }
          else 
          {
            data->status = ExtractionImageWidget::ConvertStatus(reader_status);
          }

          // currently ignore read errors. Check visualization, when reenabling.
          if (data->status == ExtractionImageWidget::Status::SUCCESS ||
              data->status == ExtractionImageWidget::Status::DETECTION_ERROR ||
              data->status == ExtractionImageWidget::Status::IMAGE_DIMENSION_MISSMATCH) 
          {
            QMetaObject::invokeMethod(calibration_widget_,
                                      [calibration_widget = calibration_widget_, data = std::move(data)]() mutable {
              calibration_widget->AddExtractionItem(
                  new ExtractionImageWidget(std::move(data), calibration_widget->TargetVisualizer()));
            });
          }
        }
      }
      catch (std::exception& ex) {
        std::string message(ex.what());

        QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_, message]() {
          calibration_widget->EndCalibration(new CalibrationResultWidget(message));
        });
        return false;
      }
    }
    
    QMetaObject::invokeMethod(calibration_widget_,
                          [calibration_widget = calibration_widget_]() { calibration_widget->StartCalibration(); });
    try {
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
    // if (is_3D_calibration) {
    //   reconstruction = static_cast<Calibrator3D*>(calibrator.get())->Reconstruction();
    // }
    
    // do undistortion
    for(int i=0;i<cam_count;i++)
    {
      FilesystemImageReader::Options distorted_reader_options;
      distorted_reader_options.image_directory = options_.images_directory;
      distorted_reader_options.image_read_mode = Pixmap::ReadMode::COLOR_AS_SOURCE;
      // distorted_reader_options.cam_index = options_.current_index;//liheng1
      distorted_reader_options.cam_index = i;//liheng1
      FilesystemImageReader distorted_reader(distorted_reader_options);
      int width, height;
      try {
        width = distorted_reader.ImagesWidth();
        height = distorted_reader.ImagesHeight();
      }
      catch (std::exception& ex) {
        QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_]() {
          calibration_widget->EndCalibration(new CalibrationResultWidget("No readable images in provided directory in undistortion!"));
        });
        return false;
      }
      colmap::Camera& distorted_camera = calibration.Camera(i);
      UndistortImages(distorted_reader, distorted_camera, options_.output_images_directory);
    }
    calibration.recomputeflag();
    // do extraction again, for extrinsic calibration
    for(int i=0;i<cam_count;i++)
        {
          int image_detected=0;
          FilesystemImageReader::Options reader_options;
          reader_options.image_directory = options_.output_images_directory;
          reader_options.cam_index = i;//liheng1
          FilesystemImageReader reader(reader_options);    
          try {
            while (reader.HasNext()) {
              Image image;
              image.issystem();
              image.is_undistort();
              std::unique_ptr<Pixmap> pixmap = std::make_unique<Pixmap>();
              ImageReader::Status reader_status = reader.Next(image, *pixmap);
              FeatureExtractor::Status extractor_status;
              std::unique_ptr<ExtractionImageWidget::Data> data = std::make_unique<ExtractionImageWidget::Data>();
              if (reader_status == ImageReader::Status::SUCCESS) 
              {
                data->image_name = image.Name();
                data->system=true;
                extractor_status = extractor->Extract(image, *pixmap);
                if (extractor_status == FeatureExtractor::Status::SUCCESS) 
                { 
                  
                  size_t id = calibration.AddImage_cam_undistorted(image,i);
                  data->image_data = calibration.Image_cam_undistorted(id,i);
                  image_detected+=1;
                }
                // else if(extractor_status == FeatureExtractor::Status::LACK_ERROR)  {
                //   throw std::runtime_error("Undistorted Image:LACK_ERROR .");
                // }
                // else{
                //   throw std::runtime_error("Undistorted Image:DETECTED ERROR .");
                // }
                else{
                  continue;
                }


                // // save a copy of the last image for the offset visualization
                // last_pixmap_ = std::make_unique<Pixmap>(pixmap->Clone());
                // data->image = std::move(pixmap);
                // data->status = ExtractionImageWidget::ConvertStatus(extractor_status);
              }
              else 
              {
                data->status = ExtractionImageWidget::ConvertStatus(reader_status);
              }
            }
            if(image_detected<1)
            {
              throw std::runtime_error("Undistorted Image:DETECTED ERROR .");
            }
          }
          catch (std::exception& ex) {
            std::string message(ex.what());

            QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_, message]() {
              calibration_widget->EndCalibration(new CalibrationResultWidget(message));
            });
            return false;
          }
        }
    try {
      calibrator->Calibrate(calibration);
    }
    catch (std::exception& ex) {
      std::string message(ex.what());

      QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_, message]() {
        calibration_widget->EndCalibration(new CalibrationResultWidget(message));
      });
      return false;
    }
    //inital extrinsic calibration




    // end calibration

    QMetaObject::invokeMethod(calibration_widget_,
                              [calibration_widget = calibration_widget_, last_pixmap = std::move(last_pixmap_), &calibration,
                                reconstruction]() mutable {
      calibration_widget->EndCalibration(new CalibrationResultWidget(calibration, std::move(last_pixmap), reconstruction));
    });

  
    return true;
  }
}
