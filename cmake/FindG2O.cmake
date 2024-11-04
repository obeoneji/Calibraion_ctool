# Findg2o.cmake

# 查找 g2o 库的头文件
find_path(G2O_INCLUDE_DIR
  NAMES g2o/core/sparse_optimizer.h
  PATHS ${G2O_ROOT_DIR}/include
  PATH_SUFFIXES g2o
)

# 查找 g2o 库文件
find_library(G2O_CORE_LIBRARY
  NAMES g2o_core
  PATHS ${G2O_ROOT_DIR}/lib
)

find_library(G2O_STUFF_LIBRARY
  NAMES g2o_stuff
  PATHS ${G2O_ROOT_DIR}/lib
)

find_library(G2O_SOLVER_DENSE_LIBRARY
  NAMES g2o_solver_dense
  PATHS ${G2O_ROOT_DIR}/lib
)

find_library(G2O_SOLVER_EIGEN_LIBRARY
  NAMES g2o_solver_eigen
  PATHS ${G2O_ROOT_DIR}/lib
)

find_library(G2O_TYPES_SLAM2D_LIBRARY
  NAMES g2o_types_slam2d
  PATHS ${G2O_ROOT_DIR}/lib
)

find_library(G2O_TYPES_SLAM3D_LIBRARY
  NAMES g2o_types_slam3d
  PATHS ${G2O_ROOT_DIR}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(G2O DEFAULT_MSG G2O_LIBRARIES G2O_INCLUDE_DIRS)

# 检查是否找到所有必需的库
if (G2O_INCLUDE_DIR AND G2O_CORE_LIBRARY AND G2O_STUFF_LIBRARY AND G2O_SOLVER_DENSE_LIBRARY AND G2O_SOLVER_EIGEN_LIBRARY  AND G2O_TYPES_SLAM2D_LIBRARY AND G2O_TYPES_SLAM3D_LIBRARY)
  set(G2O_FOUND TRUE)
else()
  set(G2O_FOUND FALSE)
endif()


# 设置库的变量
if (G2O_FOUND)
  set(G2O_LIBRARIES
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_SOLVER_DENSE_LIBRARY}
    ${G2O_SOLVER_EIGEN_LIBRARY}
    ${G2O_SOLVER_CSPARSE_LIBRARY}
    ${G2O_TYPES_SLAM2D_LIBRARY}
    ${G2O_TYPES_SLAM3D_LIBRARY}
  )
  set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
endif()

# 将库信息导出给使用者
mark_as_advanced(G2O_INCLUDE_DIR G2O_LIBRARIES)