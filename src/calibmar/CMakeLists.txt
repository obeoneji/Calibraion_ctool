set(OpenCV_DIR "C:/Users/Mayn/work/3rdparty/opencv/install")
find_package(OpenCV  REQUIRED)
message("OpenCV_INCLUDE_DIRS is found:"${OpenCV_INCLUDE_DIRS})

message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")
add_library(calibmar version.h)
add_subdirectory(calibrators)
add_subdirectory(core)
add_subdirectory(extractors)
add_subdirectory(pose_suggestion)
add_subdirectory(readers)

find_package(g2o CONFIG REQUIRED)

include_directories(${G2O_INCLUDE_DIRS})
message(STATUS "G2O_INCLUDE_DIRS = ${G2O_INCLUDE_DIRS}")
# message(STATUS "G2O_LIBRARIES = ${G2O_LIBRARIES}")


# 链接库
target_link_libraries(calibmar PUBLIC colmap  ${OpenCV_LIBS} 
g2o::freeglut_minimal 
g2o::stuff 
g2o::opengl_helper 
g2o::core 
g2o::types_data 
g2o::types_slam2d 
g2o::types_slam3d 
g2o::types_sba 
g2o::types_sim3 
g2o::types_icp 
g2o::types_sclam2d 
g2o::types_slam2d_addons 
g2o::types_slam3d_addons 
g2o::solver_pcg 
g2o::solver_dense 
g2o::solver_structure_only
g2o::solver_csparse 
g2o::csparse_extension 
g2o::solver_slam2d_linear 
g2o::solver_cholmod 
g2o::solver_eigen)
target_include_directories(calibmar PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/.. ${OpenCV_INCLUDE_DIRS}  ${G2O_INCLUDE_DIRS})
target_compile_features(calibmar PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(calibmar PUBLIC "/bigobj")
endif()

find_package(CUDAToolkit QUIET)
if(CUDA_ENABLED AND CUDAToolkit_FOUND) # also use the colmap cuda enabled option
    target_compile_definitions(calibmar PUBLIC COLMAP_CUDA_ENABLED)
    message(STATUS "Colmap CUDA is enabled")
else()
    message(STATUS "Colmap CUDA is NOT enabled")
endif()

include(build_info)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/version.h.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/version.h")