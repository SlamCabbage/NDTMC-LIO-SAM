#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"
// #include "utility.h"

#include "tictoc.h"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;
using std::vector;
using PointT = pcl::PointXYZI;

class NdtMCManager
{
protected:
    
    /** \brief Typename of searchable voxel grid containing mean and covariance. */
    using NDTGrid = typename pcl::VoxelGridCovariance<PointT>;
    /** \brief Typename of VoxelGridCovariance leaf structure */
    using Leaf = typename NDTGrid::Leaf;
    /** \brief Pointer to VoxelGridCovariance leaf structure */
    using LeafPtr = Leaf *;
    /** \brief Const pointer to VoxelGridCovariance leaf structure */
    using LeafConstPtr = const Leaf *;

    using KeyMat = std::vector<std::vector<float>>;
    using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

public:
    struct NDTCell
    {
        NDTCell(Eigen::Vector3d mean_, Eigen::Matrix3d cov) : cov_(cov)
        {
            point_.x = mean_(0);
            point_.y = mean_(1);
            point_.z = mean_(2);
        }
        PointT point_;
        Eigen::Matrix3d cov_ = Eigen::Matrix3d::Identity();
    };
    using NDTCellPtr = NDTCell *;
    struct GridCell {
        int counts[8] = {0}; // 记录每个shape出现的次数
        int shape_max = -1;  // 记录出现次数最多的shape
        int count_max = 0;   // 记录出现次数最多的shape出现的次数

        void addShape(int shape) {
            counts[shape - 1]++;  // 根据shape值更新counts数组
            int total_counts = 0;
            for (int count : counts) {
                total_counts += count;
            }
            if (counts[shape - 1] > count_max && counts[shape - 1] >= double(total_counts) / 8.0) {  // 如果出现次数超过了之前最大值并且超过总次数的25%
                count_max = counts[shape - 1];
                shape_max = shape;
            } else if (count_max < double(total_counts) / 8.0) {  // 如果最多的shape数量没有超过25%
                shape_max = -1;
            }
        }
    };

public:
    /** \brief Constructor.
     * initialize \ref global_ndt_, \ref submap_ndt_ and \ref frame_ndt_
     * set \ref falcon_To_car
     */
    NdtMCManager() : ndt_pointcloud(new pcl::PointCloud<PointT>()) {};

    float rad2deg_(float radians);

    float deg2rad_(float degrees);

    const Eigen::MatrixXd &getConstRefRecentSCD(void);

    double distDirectSC(MatrixXd &_sc1, MatrixXd &_sc2);

    double distDirectNDTMC(const Eigen::MatrixXd &_frame_sc, Eigen::MatrixXd _submap_sc);

    MatrixXd circshift(MatrixXd &_mat, int _num_shift);

    int fastAlignUsingVkey(MatrixXd &_vkey1, MatrixXd &_vkey2);

    std::pair<double, int> distanceBtnScanContext(MatrixXd &_sc1, MatrixXd &_sc2);

   std::pair<double, int> distanceBtnNDTScanContext(Eigen::MatrixXd &frame_sc,
                                                           Eigen::MatrixXd &submap_sc);

    std::pair<int, float> detectLoopClosureID(pcl::PointCloud<pcl::PointXYZI>::Ptr copy_cloudKeyPoses3D);

    std::vector<float> eig2stdvec(MatrixXd _eigmat);

    Eigen::MatrixXd circshiftNDTMC(Eigen::MatrixXd _mat);

    Eigen::MatrixXd makeSectorkeyFromNdtMapCode(Eigen::MatrixXd &_desc);

    Eigen::MatrixXd makeRingkeyFromNdtMapCode(Eigen::MatrixXd &_desc);

    int GetDistributionWeight(int idx);

    int CalculateLayer(float z) const;

    float xy2theta(const float &_x, const float &_y);

    bool CalculateSectorId(PointT &pt_in, int &ring_idx, int &sctor_idx);

    Eigen::MatrixXd getNDTLeaves(const std::map<std::size_t, Leaf> &leaves_);

    void transformToNDTForm(float resolution_, pcl::PointCloud<PointT> &cloud_in);

    MatrixXd makeScancontext( pcl::PointCloud<PointT> & _scan_down );

    void makeAndSaveNdtMapCodeAndKeys(pcl::PointCloud<PointT> cloud_in);

public:
    // parameter of scancontext
    const float LIDAR_HEIGHT = 1.73;
    const int PC_NUM_RING = 20;    // kitti ---> 20  nio ---> 40
    const int PC_NUM_SECTOR = 60; // kitti ---> 60  nio ---> 180
    const int PC_NUM_Z = 6;

    const double PC_MAX_RADIUS = 80.0;
    static constexpr double PC_MAX_Z = 6;
    const int NO_POINT = -1000;
    const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    
    // tree
    static const int num_intervals = 10;
    Eigen::Matrix<double,1,num_intervals> histogram;
    const int NUM_EXCLUDE_RECENT = 30;       // simply just keyframe gap (related with loopClosureFrequency in yaml), but node position distance-based exclusion is ok.
    const int NUM_CANDIDATES_FROM_TREE = 10; // 10 is enough. (refer the IROS 18 paper)

    // loop thres
    const double SEARCH_RATIO = 0.3; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    // const double SC_DIST_THRES = 0.13;  // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
    const double SC_DIST_THRES = 0.7; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15
    // const double SC_DIST_THRES = 0.7; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

    // config
    const int TREE_MAKING_PERIOD_ = 1; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / in the LeGO-LOAM integration, it is synchronized with the loop detection callback (which is 1Hz) so it means the tree is updated evrey 10 sec. But you can use the smaller value because it is enough fast ~ 5-50ms wrt N.
    int tree_making_period_conter = 0;

    double Similarity;

    // ndt format
    NDTGrid voxel_grid_frame;
    vector<NDTCell> ndtcell;
    pcl::PointCloud<PointT>::Ptr ndt_pointcloud;
    float resolution;

    // data
    std::vector<double> polarcontexts_timestamp_; // optional.
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;
};