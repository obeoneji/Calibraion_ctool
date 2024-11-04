#include "colmap_calibration.h"

#include "colmap/controllers/automatic_reconstruction.h"
#include "colmap/controllers/feature_matching.h"
#include "colmap/controllers/incremental_mapper.h"
#include "colmap/geometry/triangulation.h"
#include "colmap/scene/projection.h"

#include "calibmar/calibrators/general_calibration.h"

namespace {
  std::unique_ptr<colmap::Database> PrepareColmapDB(calibmar::Calibration& calibration, const std::string& database_path,
                                                    std::map<colmap::image_t, size_t>& colmap_img_to_calib_img) {
    std::unique_ptr<colmap::Database> database = std::make_unique<colmap::Database>(database_path);
    // this is neccessary because Windows will keep the DB around for the complete process (and potentially longer?)
    // To avoid key constraint exceptions (i.e. inserting the same object ids multiple times) ensure the DB is clean from the
    // start...
    database->ClearAllTables();

    colmap::DatabaseTransaction database_transaction(database.get());

    calibration.Camera().camera_id = database->WriteCamera(calibration.Camera());

    for (size_t i = 0; i < calibration.Images().size(); i++) {
      const calibmar::Image& image = calibration.Images()[i];
      colmap::Image colmap_img;
      colmap_img.SetCameraId(calibration.Camera().camera_id);
      colmap_img.SetName(image.Name());

      colmap::image_t img_id = database->WriteImage(colmap_img);

      colmap_img_to_calib_img.emplace(img_id, i);
      database->WriteKeypoints(img_id, image.FeatureKeypoints());
      database->WriteDescriptors(img_id, image.FeatureDescriptors());
    }
    return std::move(database);
  }

  struct ImageStats {
    double per_img_rms;
    int per_img_3d_point_count;
  };

  void CalculatePerViewRms(colmap::Reconstruction& reconstruction, std::map<colmap::image_t, ImageStats>& img_stats) {
    // NOTE: The average of the per view RMS will likely not match the overall RMS reported by the reconstruction.
    // This is because the overall RMS is calculated over the 3D point tracks, while the per view is calculated over all observed
    // features. Since not all 3D points are visible in all views, the two means do not match (i.e. a outlier in view or 3D point
    // RMS will influence the respective overall RMS).
    for (const auto& id_image : reconstruction.Images()) {
      if (!id_image.second.IsRegistered()) {
        continue;
      }
      const auto& camera = reconstruction.Camera(id_image.second.CameraId());
      int num_image_pts = 0;
      double error_sum = 0;
      for (const auto& point2D : id_image.second.Points2D()) {
        if (point2D.HasPoint3D()) {
          const auto& point3D = reconstruction.Point3D(point2D.point3D_id);
          error_sum += std::sqrt(colmap::CalculateSquaredReprojectionError(
              point2D.xy, point3D.xyz, id_image.second.CamFromWorld(), camera, camera.IsCameraRefractive()));
          num_image_pts++;
        }
      }

      img_stats[id_image.first] = {error_sum / num_image_pts, (int)id_image.second.NumPoints3D()};
    }
  }
}

namespace calibmar::colmap_calibration {
  void CalibrateCamera(calibmar::Calibration& calibration, std::shared_ptr<colmap::Reconstruction>& reconstruction,
                       bool enable_refraction) {
    colmap::SiftMatchingOptions sift_options;
    colmap::TwoViewGeometryOptions geometry_options;
    colmap::IncrementalMapperOptions mapper_options;
    CalibrateCamera(calibration, reconstruction, enable_refraction, sift_options, geometry_options, mapper_options);
  }

  void CalibrateCamera(calibmar::Calibration& calibration, std::shared_ptr<colmap::Reconstruction>& reconstruction,
                       bool enable_refraction, colmap::SiftMatchingOptions& matching_options,
                       colmap::TwoViewGeometryOptions& geometry_options, colmap::IncrementalMapperOptions& mapper_options) {
    std::string database_path = "file:memory.db?mode=memory&cache=shared";
    std::map<colmap::image_t, size_t> colmap_img_to_calib_img;
    std::unique_ptr<colmap::Database> database = PrepareColmapDB(calibration, database_path, colmap_img_to_calib_img);

#if defined(COLMAP_CUDA_ENABLED)
    matching_options.use_gpu = true;
#else
    matching_options.use_gpu = false;
#endif
    colmap::ExhaustiveMatchingOptions options;

    if (enable_refraction) {
      geometry_options.enable_refraction = true;
    }

    std::unique_ptr<colmap::Thread> feature_matcher =
        colmap::CreateExhaustiveFeatureMatcher(options, matching_options, geometry_options, database_path);
    feature_matcher->Start();
    feature_matcher->Wait();

    std::shared_ptr<colmap::ReconstructionManager> reconstruction_manager = std::make_shared<colmap::ReconstructionManager>();
    std::shared_ptr<colmap::IncrementalMapperOptions> mapper_options_ptr = std::make_shared<colmap::IncrementalMapperOptions>();
    *mapper_options_ptr = mapper_options;

    mapper_options_ptr->ba_fix_intrin_until_num_images = 4;

    // For pinhole models without distortion, fix the principal point
    if (calibration.Camera().model_id == colmap::PinholeCameraModel::model_id ||
        calibration.Camera().model_id == colmap::SimplePinholeCameraModel::model_id) {
      mapper_options_ptr->ba_refine_principal_point = false;
    }
    else {
      mapper_options_ptr->ba_refine_principal_point = true;
    }

    mapper_options_ptr->extract_colors = false;

    if (enable_refraction) {
      mapper_options_ptr->enable_refraction = enable_refraction;
      mapper_options_ptr->ba_refine_refrac_params = true;
      mapper_options_ptr->ba_refine_extra_params = false;
      mapper_options_ptr->ba_refine_focal_length = false;
      mapper_options_ptr->ba_refine_principal_point = false;
    }

    std::unique_ptr<colmap::IncrementalMapperController> mapper =
        std::make_unique<colmap::IncrementalMapperController>(mapper_options_ptr,
                                                              /*image_path=*/"", database_path, reconstruction_manager);
    mapper->Start();
    mapper->Wait();

    if (reconstruction_manager->Size() <= 0) {
      throw std::runtime_error("Reconstruction unsuccessful!");
    }
    reconstruction = reconstruction_manager->Get(0);

    calibration.SetCamera(reconstruction->Camera(calibration.Camera().camera_id));
    calibration.SetCalibrationRms(reconstruction->ComputeMeanReprojectionError());

    std::map<colmap::image_t, ImageStats> per_img_stats;
    CalculatePerViewRms(*reconstruction, per_img_stats);
    std::vector<double> per_view_rms(calibration.Images().size(), -1.0);
    std::vector<int> per_view_3d_points(calibration.Images().size(), -1);
    for (const auto& id_stats : per_img_stats) {
      per_view_rms[colmap_img_to_calib_img.at(id_stats.first)] = id_stats.second.per_img_rms;
      per_view_3d_points[colmap_img_to_calib_img.at(id_stats.first)] = id_stats.second.per_img_3d_point_count;
    }
    calibration.SetPerViewRms(per_view_rms);
    calibration.SetPerView3DPoints(per_view_3d_points);

    for (auto id : reconstruction->RegImageIds()) {
      const colmap::Rigid3d& pose = reconstruction->Image(id).CamFromWorld();
      calibmar::Image& img = calibration.Image(colmap_img_to_calib_img[id]);
      img.SetPose(pose);
      // remove all registered images from the map to see if all images were registered
      colmap_img_to_calib_img.erase(id);
    }

    // if unregistered images remain, throw an error
    if (colmap_img_to_calib_img.size() > 0.3 * calibration.Images().size()) {
      std::stringstream unregistered_images;
      for (const auto& col_calib : colmap_img_to_calib_img) {
        unregistered_images << "\n" << calibration.Image(col_calib.second).Name();
      }
      throw std::runtime_error(colmap::StringPrintf("More than 30%% of images (%d/%d) not registered!\nImage(s):\n%s",
                                                    colmap_img_to_calib_img.size(), calibration.Images().size(),
                                                    unregistered_images.str().c_str()));
    }
  }
}