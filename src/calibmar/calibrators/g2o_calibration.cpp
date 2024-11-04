#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <stdint.h>
#include <cassert>
#include <iostream>
#include <unordered_set>
#include "g2o/config.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "calibmar/core/calibration.h"
#include "calibmar/core/camera_models.h"
#include <opencv2/core/mat.hpp>
class Sample {
 public:
  static int uniform(int from, int to) {
    return static_cast<int>(g2o::Sampler::uniformRand(from, to));
  }
};
using namespace Eigen;
using namespace std;
#if defined G2O_HAVE_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#endif
// #if defined G2O_HAVE_CHOLMOD
// G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
// #else
// G2O_USE_OPTIMIZATION_LIBRARY(eigen);
// #endif
G2O_USE_OPTIMIZATION_LIBRARY(dense);
namespace calibmar::g2o_calibration {
    void testG2o(Calibration& calibration,int cam_num, int root_cam,std::vector<std::vector<Eigen::Vector2d>>&Cam_point2dSets,
    std::vector<std::vector<Eigen::Vector3d>>&Cam_point3dSets,std::vector<std::vector<int>>&Cam_charucoidSets,std::vector<Eigen::Vector3d>&point3d_dict,
    std::vector<int>point3d_dict_ids,std::vector<cv::Mat>&rotations,std::vector<cv::Mat>& translations,int solver_index) 
    {
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
        if (solver_index==2) {
            linearSolver = std::make_unique<
                g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
            cerr << "Using DENSE" << endl;
        } 
        else if(solver_index==1) 
        {
            cerr << "Using CHOLMOD" << endl;
            linearSolver = std::make_unique<
                g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
        }
        else
        {
            linearSolver = std::make_unique<
                g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
            cerr << "Using CSPARSE" << endl;

        }

        g2o::OptimizationAlgorithmLevenberg* solver =
            new g2o::OptimizationAlgorithmLevenberg(
                std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

        optimizer.setAlgorithm(solver);
    // //define solver 
    //     g2o::SparseOptimizer optimizer;
    //     optimizer.setVerbose(false);
    //     std::string solverName;


    //	if (DENSE) {
    // solverName = "lm_dense6_3";
    //	} else {
    //#ifdef G2O_HAVE_CHOLMOD
    //		solverName = "lm_fix6_3_cholmod";
    //#else
    //		solverName = "lm_fix6_3";
    //#endif
    //	}

        // g2o::OptimizationAlgorithmProperty solverProperty;
        // optimizer.setAlgorithm(
        //         g2o::OptimizationAlgorithmFactory::instance()->construct(solverName,solverProperty));
	    std::vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat> > true_poses;
    //define cameras and poses(which will be optimizied)
        for(int i=0;i<cam_num;i++)
        {        
            const colmap::Camera& camera_undistorted = calibration.Camera_undistorted(i); 
            double focal_length = camera_undistorted.params[0]; 
            Eigen::Vector2d principal_point(camera_undistorted.params[2], camera_undistorted.params[3]);

            g2o::CameraParameters* cam_params =
                new g2o::CameraParameters(focal_length, principal_point, 0.);
            cam_params->setId(i*2);
            if(!optimizer.addParameter(cam_params)) assert(false); 
            cv::Mat pose_cv=calibration.InitalPose(i);
            cv::Mat rot_cv=pose_cv(cv::Rect(0, 0, 3, 3)).clone(); // 复制 3x3 矩阵
            cv::Mat trans_cv = pose_cv(cv::Rect(3, 0, 1, 3)).clone(); // 复制 3x1 矩阵
            Eigen::Matrix3d rot_eigen;
            cv::cv2eigen(rot_cv, rot_eigen); 
            // 从旋转矩阵创建四元数
            Eigen::Quaterniond q(rot_eigen);

            Eigen::Vector3d trans;
            trans << trans_cv.at<double>(0, 0), trans_cv.at<double>(1, 0), trans_cv.at<double>(2, 0);
   		    g2o::SE3Quat pose(q, trans);
            g2o::VertexSE3Expmap* v_se3 = new g2o::VertexSE3Expmap();
            v_se3->setId(i);
            if (i==root_cam-1) 
            {
                v_se3->setFixed(true);
            }
            else{
                v_se3->setFixed(false);         
            }
            v_se3->setEstimate(pose);
            optimizer.addVertex(v_se3);
            true_poses.push_back(pose);

            //Set the poses that should be optimized.
            //Define their initial value to be the true pose
            //keep in mind that there is added noise to the observations afterwards.
        }
    //define points and observations
        for(int i=0;i<point3d_dict_ids.size();i++)
        {
            int point_id=point3d_dict_ids[i];
            int count_num=0;
            for (int j=0;j<cam_num;j++)
            {
                auto it = std::find(Cam_charucoidSets[j].begin(), Cam_charucoidSets[j].end(), point_id);
                if(it!=Cam_charucoidSets[j].end())
                {
                    count_num++;
                }
            }
            if(count_num>1)
            {   //define points
                g2o::VertexPointXYZ* v_p = new g2o::VertexPointXYZ();
		        v_p->setId(point_id*2+1);
                v_p->setMarginalized(false); 
                v_p->setFixed(false);
                v_p->setEstimate(point3d_dict[point_id]);
                optimizer.addVertex(v_p);
                //define observation
                for (int j=0;j<cam_num;j++)
                {
                    auto it = std::find(Cam_charucoidSets[j].begin(), Cam_charucoidSets[j].end(), point_id);
                    if(it != Cam_charucoidSets[j].end())
                    {   
                        int index = std::distance(Cam_charucoidSets[j].begin(), it);
                        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
                        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
                        edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertices().find(j)->second));
                        edge->setMeasurement(Cam_point2dSets[j][index]);
                        edge->information() = Eigen::Matrix2d::Identity()* 0.1;
                        edge->setRobustKernel(new g2o::RobustKernelHuber);
                        edge->setParameterId(0, j*2);
                        optimizer.addEdge(edge);
                    }
                }
            }
        }


    //define optimization




        optimizer.initializeOptimization();
        optimizer.setVerbose(false);
        optimizer.save("before.g2o");
	    optimizer.optimize(100);
        optimizer.save("after.g2o");
        for(int i=0;i<cam_num;i++)
        {           
            g2o::VertexSE3Expmap* vertex = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertices().find(i)->second);
            if (vertex == 0) {
                continue;
            }
            Eigen::Quaterniond q = vertex->estimate().rotation();
            Eigen::Vector3d t_ = vertex->estimate().translation();



            Eigen::Matrix3d R_ = q.toRotationMatrix(); // 或者 q.normalized().toRotationMatrix();

            // 转换为 cv::Mat
            cv::Mat R_cv;
            cv::Mat t_cv;

            // 将 Eigen::Matrix3d 转换为 cv::Mat
            cv::eigen2cv(R_, R_cv);
            // 将 Eigen::Vector3d 转换为 cv::Mat
            cv::eigen2cv(t_, t_cv);
            rotations[i]=R_cv;
            translations[i]=t_cv;
        }
        // optimizer.clear();
        return;
    }

    //define points

}