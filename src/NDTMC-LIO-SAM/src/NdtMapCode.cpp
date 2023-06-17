#include "NdtMapCode.h"

float NdtMCManager::rad2deg_(float radians)
{
    return radians * 180.0 / M_PI;
}

float NdtMCManager::deg2rad_(float degrees)
{
    return degrees * M_PI / 180.0;
}

const Eigen::MatrixXd &NdtMCManager::getConstRefRecentSCD(void)
{
    return polarcontexts_.back();
}

double NdtMCManager::distDirectSC(MatrixXd &_sc1, MatrixXd &_sc2)
{
    int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    for (int col_idx = 0; col_idx < _sc1.cols(); col_idx++)
    {
        VectorXd col_sc1 = _sc1.col(col_idx);
        VectorXd col_sc2 = _sc2.col(col_idx);

        if ((col_sc1.norm() == 0) | (col_sc2.norm() == 0))
            continue; // don't count this sector pair.

        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }

    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;

} // distDirectSC

MatrixXd NdtMCManager::circshift(MatrixXd &_mat, int _num_shift)
{
    // shift columns to right direction
    assert(_num_shift >= 0);

    if (_num_shift == 0)
    {
        MatrixXd shifted_mat(_mat);
        return shifted_mat; // Early return
    }

    MatrixXd shifted_mat = MatrixXd::Zero(_mat.rows(), _mat.cols());
    for (int col_idx = 0; col_idx < _mat.cols(); col_idx++)
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

} // circshift

int NdtMCManager::fastAlignUsingVkey(MatrixXd &_vkey1, MatrixXd &_vkey2)
{
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    for (int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++)
    {
        MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

        MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

        double cur_diff_norm = vkey_diff.norm();
        if (cur_diff_norm < min_veky_diff_norm)
        {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }

    return argmin_vkey_shift;

} // fastAlignUsingVkey

double NdtMCManager::distDirectNDTMC(const Eigen::MatrixXd &_frame_sc, Eigen::MatrixXd _submap_sc)
{
    // Calculate a_mean and b_mean
    double a_mean = _frame_sc.mean();
    double b_mean = _submap_sc.mean();

    // Center a_mix and b_mix around their means
    Eigen::MatrixXd a = _frame_sc.array() - a_mean;
    a = (a.array() == -a_mean).select(0, a); // replace -a_mean with 0
    Eigen::MatrixXd b = _submap_sc.array() - b_mean;
    b = (b.array() == -b_mean).select(0, b); // replace -b_mean with 0
    // const Eigen::MatrixXd &a = _frame_sc;
    // const Eigen::MatrixXd &b = _submap_sc;

    // // Calculate correlation similarity
    double sc_sim = (a.cwiseProduct(b).colwise().sum()).sum() /
                    sqrt((a.cwiseProduct(a).colwise().sum()).sum() * (b.cwiseProduct(b).colwise().sum()).sum());

    return 1.0 - abs(sc_sim);

} // distDirectNDTMC

Eigen::MatrixXd NdtMCManager::circshiftNDTMC(Eigen::MatrixXd _mat)
{
    Eigen::MatrixXd expanded_mat = Eigen::MatrixXd::Zero(_mat.rows(), 2 * _mat.cols() - 1);
    expanded_mat.block(0, 0, _mat.rows(), _mat.cols()) = _mat;
    for (int i = 1; i < _mat.cols(); ++i)
    {
        expanded_mat.block(0, i + _mat.cols() - 1, _mat.rows(), 1) = _mat.block(0, i, _mat.rows(), 1);
    }
    return expanded_mat;
} // circshift

std::pair<double, int> NdtMCManager::distanceBtnNDTScanContext(Eigen::MatrixXd &frame_sc,
                                                           Eigen::MatrixXd &submap_sc)
{
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    // std::vector<std::pair<double, int>> results;
    // results.emplace_back(1000.0, -1);
    int len_cols = static_cast<int>(submap_sc.cols());
    //    Eigen::MatrixXd submap_sc_ = circshift(submap_sc, frame_sc);
    Eigen::MatrixXd submap_sc_ = circshiftNDTMC(submap_sc);
    for (int num_shift = 0; num_shift < len_cols; ++num_shift)
    {
        Eigen::MatrixXd submap_sc_shifted = submap_sc_.block(0, num_shift, frame_sc.rows(), frame_sc.cols());
        double cur_sc_dist = distDirectNDTMC(frame_sc, submap_sc_shifted);
        if (cur_sc_dist == 0)
        {
            continue;
        }
        if (cur_sc_dist != cur_sc_dist)
            continue;
        if (cur_sc_dist < min_sc_dist)
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
        // results.emplace_back(cur_sc_dist, num_shift);
    }
    return make_pair(min_sc_dist, argmin_shift);
} // distanceBtnNDTScanContext

std::pair<double, int> NdtMCManager::distanceBtnScanContext(MatrixXd &_sc1, MatrixXd &_sc2)
{
    // 1. fast align using variant key (not in original IROS18)
    MatrixXd vkey_sc1 = makeSectorkeyFromNdtMapCode(_sc1);
    MatrixXd vkey_sc2 = makeSectorkeyFromNdtMapCode(_sc2);
    int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

    const int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * _sc1.cols()); // a half of search range
    std::vector<int> shift_idx_search_space{argmin_vkey_shift};
    for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++)
    {
        shift_idx_search_space.push_back((argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());
        shift_idx_search_space.push_back((argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for (int num_shift : shift_idx_search_space)
    {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        // double cur_sc_dist = distDirectSC(_sc1, sc2_shifted);
        double cur_sc_dist = distDirectNDTMC(_sc1, sc2_shifted);
        if (cur_sc_dist < min_sc_dist)
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    return make_pair(min_sc_dist, argmin_shift);

} // distanceBtnScanContext

std::pair<int, float> NdtMCManager::detectLoopClosureID(pcl::PointCloud<pcl::PointXYZI>::Ptr copy_cloudKeyPoses3D)
{
    int loop_id{-1}; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")

    auto curr_key = polarcontext_invkeys_mat_.back(); // current observation (query)
    auto curr_desc = polarcontexts_.back();           // current observation (query)

    /*
     * step 1: candidates from ringkey tree_
     */
    if ((int)polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
    {
        std::pair<int, float> result{loop_id, 0.0};
        return result; // Early return
    }

    // tree_ reconstruction (not mandatory to make everytime)
    if (tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost
    {
        TicToc t_tree_construction;

        polarcontext_invkeys_to_search_.clear();
        polarcontext_invkeys_to_search_.assign(polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT);

        polarcontext_tree_.reset();
        polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */);
        // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
        t_tree_construction.toc("Tree construction");
    }
    tree_making_period_conter = tree_making_period_conter + 1;

    double min_dist = 10000000; // init with somthing large
    int nn_align = 0;
    int nn_idx = 0;

    // knn search
    std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE);
    std::vector<float> out_dists_sqr(NUM_CANDIDATES_FROM_TREE);

    TicToc t_tree_search;
    nanoflann::KNNResultSet<float> knnsearch_result(NUM_CANDIDATES_FROM_TREE);
    knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
    polarcontext_tree_->index->findNeighbors(knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10));
    t_tree_search.toc("Tree search");

    /*
     *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
     */
    int num_of_keyframe = copy_cloudKeyPoses3D->points.size();
    pcl::PointXYZI current_pose3d = copy_cloudKeyPoses3D->points[num_of_keyframe-1];
    TicToc t_calc_dist;
    static int ll;
    if (num_of_keyframe % 50 == 0)
    {
        ll = 0;
    }
    else
    {
        ll++;
    }
    for (int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++)
    {
        // TODO: 进行欧式距离判断，不对距离过大的回环检测进行判断
        pcl::PointXYZI loop_pose3d = copy_cloudKeyPoses3D->points[candidate_indexes[candidate_iter_idx]];
        
        double odom_distance = sqrt((loop_pose3d.x-current_pose3d.x) * (loop_pose3d.x-current_pose3d.x) + (loop_pose3d.y-current_pose3d.y) * (loop_pose3d.y-current_pose3d.y));
        if (odom_distance > 30 + double(ll) / 20.0)
        {
            // std::pair<int, float> result{loop_id, 0.0};
            // return result; // Early return
            continue;
        }

        MatrixXd polarcontext_candidate = polarcontexts_[candidate_indexes[candidate_iter_idx]];
        std::pair<double, int> sc_dist_result = distanceBtnScanContext(curr_desc, polarcontext_candidate);

        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

        if (candidate_dist < min_dist)
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;

            nn_idx = candidate_indexes[candidate_iter_idx];
        }
    }
    t_calc_dist.toc("Distance calc");

    /*
     * loop threshold check
     */
    if (min_dist < SC_DIST_THRES)
    {
        loop_id = nn_idx;

        // std::cout.precision(3);
        cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size() - 1 << " and " << nn_idx << " yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }
    else
    {
        std::cout.precision(3);
        cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_.size() - 1 << " and " << nn_idx << " yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }

    // To do: return also nn_align (i.e., yaw diff)
    float yaw_diff_rad = deg2rad_(nn_align * PC_UNIT_SECTORANGLE);
    std::pair<int, float> result{loop_id, yaw_diff_rad};

    return result;

} // detectLoopClosureID

std::vector<float> NdtMCManager::eig2stdvec(MatrixXd _eigmat)
{
    std::vector<float> vec(_eigmat.data(), _eigmat.data() + _eigmat.size());
    return vec;
} // eig2stdvec

Eigen::MatrixXd NdtMCManager::makeSectorkeyFromNdtMapCode(Eigen::MatrixXd &_desc)
{
    /*
     * summary: columnwise mean vector
     */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for (int col_idx = 0; col_idx < _desc.cols(); col_idx++)
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
} // makeSectorkeyFromNdtMapCode


int NdtMCManager::GetDistributionWeight(int idx)
{
    NDTCellPtr cell_temp = nullptr;
    cell_temp = &ndtcell[idx];
    // 获取cov的特征向量，转化至局部坐标系，判断方向
    Eigen::Matrix3d cov = cell_temp->cov_;
    // AVP_LOGI << cov << "\n" << idx;
    // Eigen::Quaterniond quaternion(q_w, q_x, q_y, q_z);
    // cov = quaternion * cov * quaternion.conjugate();
    cov.normalize();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov); // SelfAdjointEigenSolver计算特征向量和特征值
    double value = saes.eigenvalues()(2) * saes.eigenvalues()(0) / (saes.eigenvalues()(1) * saes.eigenvalues()(1));
    Eigen::Vector3d plane_normal_vector = saes.eigenvectors().col(0);
    Eigen::Vector3d xnormal;
    xnormal << 0, 0, 1;
    // double angle = acos(plane_normal_vector.dot(xnormal) / (plane_normal_vector.norm())) / M_PI * 180;
    if (value <= 0.6)
    {
        return 0;
    }
    else if (value > 0.6 && value <= 3)
    {
        return 1;
    }
    else if (value > 3 && value <= 6)
    {
        return 2;
    }
    return -1;
}


float NdtMCManager::xy2theta(const float &_x, const float &_y)
{
    if ((_x >= 0) & (_y >= 0))
        return (180 / M_PI) * atan(_y / _x);

    if ((_x < 0) & (_y >= 0))
        return 180 - ((180 / M_PI) * atan(_y / (-_x)));

    if ((_x < 0) & (_y < 0))
        return 180 + ((180 / M_PI) * atan(_y / _x));

    if ((_x >= 0) & (_y < 0))
        return 360 - ((180 / M_PI) * atan((-_y) / _x));
} // xy2theta

bool NdtMCManager::CalculateSectorId(PointT &pt_in, int &ring_idx, int &sctor_idx)
{
    // xyz to ring, sector
    float azim_range = sqrt(pt_in.x * pt_in.x + pt_in.y * pt_in.y);
    float azim_angle = xy2theta(pt_in.x, pt_in.y);
    // if range is out of roi, pass
    if (azim_range > PC_MAX_RADIUS)
        return false;

    ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
    sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);
    return true;
}

MatrixXd NdtMCManager::makeScancontext(pcl::PointCloud<PointT> &_scan_down)
{
    TicToc t_making_desc;

    int num_pts_scan_down = _scan_down.points.size();

    // main
    const int NO_POINT = -1000;
    MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

    PointT pt;
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x;
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if (azim_range > PC_MAX_RADIUS)
            continue;

        ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
        sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);

        // taking maximum z
        if (desc(ring_idx - 1, sctor_idx - 1) < pt.z) // -1 means cpp starts from 0
            desc(ring_idx - 1, sctor_idx - 1) = pt.z; // update for taking maximum value at that bin
    }

    // reset no points to zero (for cosine dist later)
    for (int row_idx = 0; row_idx < desc.rows(); row_idx++)
        for (int col_idx = 0; col_idx < desc.cols(); col_idx++)
            if (desc(row_idx, col_idx) == NO_POINT)
                desc(row_idx, col_idx) = 0;

    t_making_desc.toc("PolarContext making");

    return desc;
} // makeScancontext

int NdtMCManager::CalculateLayer(float z) const
{
    if (z < 0 || z > PC_MAX_Z)
    {
        return -1;
    }
    int layer = std::max(std::min(PC_NUM_Z, int(ceil((z / PC_MAX_Z) * PC_NUM_Z))), 1);
    //    int layer = static_cast<int>(std::floor(z / PC_LAYER)) + 1;

    return layer;
}

Eigen::MatrixXd NdtMCManager::getNDTLeaves(const std::map<std::size_t, Leaf> &leaves_)
{
    GridCell grid[PC_NUM_RING][PC_NUM_SECTOR][PC_NUM_Z];
    Eigen::MatrixXd desc_1(PC_NUM_RING, PC_NUM_SECTOR);
    desc_1.setZero();
    histogram.setZero();
    for (const auto &leave : leaves_)
    {
        if (leave.second.nr_points < 5 || leave.second.evals_(2) == 0)
        {
            // cout << leave.second.nr_points << " " << leave.second.evals_(2) << endl;
            continue;
        }

        // 1. 获取cov和mean
        Eigen::Matrix3d cov = leave.second.cov_;
        pcl::PointXYZI point;
        point.x = static_cast<float>(leave.second.mean_(0));
        point.y = static_cast<float>(leave.second.mean_(1));
        point.z = static_cast<float>(leave.second.mean_(2));
        // 2. 获取ring_id,sector_id
        int ring_idx, sector_idx;
        // 1. 根据xy的值计算它属于哪一个bin
        if (!CalculateSectorId(point, ring_idx, sector_idx))
        {
            continue;
        }
        // 3. 根据z值计算它所在的层
        //        weight = CalculateLayerWeight(pt.z);
        int z_idx = CalculateLayer(point.z);
        if (z_idx == -1)
        {
            continue;
        }
        double value = leave.second.evals_(2) * leave.second.evals_(0) / (leave.second.evals_(1) * leave.second.evals_(1));

        double interval_size = 3 / double(num_intervals);
        if (value < 3)
        {
            int his_idx = static_cast<int>(std::floor(value / interval_size));
            histogram(0, his_idx)++;
        }

        if (value > 0 && value < 2.4)
        {
            int shape_id = static_cast<int>(std::floor(value / 0.3));
            grid[ring_idx - 1][sector_idx - 1][z_idx - 1].addShape(shape_id + 1);
            double N = 3.0;
            desc_1(ring_idx - 1, sector_idx - 1) += ((N / 2) * (1 + log(2 * M_PI)) + 0.5 * log(cov.determinant())) * z_idx / PC_NUM_Z;
        }
        else
        {
            continue;
        }
    }

    //     Compute NDTMC matrix using shape weights
    Eigen::MatrixXd desc_2(PC_NUM_RING, PC_NUM_SECTOR);
    desc_2.setZero();
    for (int i = 0; i < PC_NUM_RING; i++)
    {
        for (int j = 0; j < PC_NUM_SECTOR; j++)
        {
            double weight = 0;
            for (int k = 0; k < PC_NUM_Z; k++)
            {
                if (grid[i][j][k].shape_max == -1)
                {
                    continue;
                }
                weight += double(grid[i][j][k].shape_max * (k + 1)) / PC_NUM_Z;
                // weight += double(grid[i][j][k].shape_max) / PC_NUM_Z; //avarage pooling
            }
            desc_2(i, j) = weight;
        }
    }
    Eigen::MatrixXd desc(2 * PC_NUM_RING, PC_NUM_SECTOR);
    desc << desc_1,
        desc_2;
    return desc;
} // getNDTLeaves

void NdtMCManager::transformToNDTForm(float resolution_, pcl::PointCloud<PointT> &cloud_in)
{
    for (int i = 0; i < cloud_in.size(); ++i)
    {
        cloud_in.points[i].z += LIDAR_HEIGHT;
    }
    pcl::PointCloud<PointT>::Ptr cloudPtr = boost::make_shared<pcl::PointCloud<PointT>>(cloud_in);
    voxel_grid_frame.setLeafSize(resolution_, resolution_, resolution_);
    voxel_grid_frame.setInputCloud(cloudPtr);
    // Initiate voxel structure.
    voxel_grid_frame.filter(true);
} // transformToNDTForm

void NdtMCManager::makeAndSaveNdtMapCodeAndKeys(pcl::PointCloud<PointT> cloud_in)
{
    // 1. transform pointcloud to NDT format
    transformToNDTForm(1.0, cloud_in);
    // cout << "cloud_in size: " << cloud_in.size() << endl;
    // 2. 获取描述子
    Eigen::Matrix ndtmc = getNDTLeaves(voxel_grid_frame.getLeaves());

    std::vector<float> polarcontext_invkey_vec = eig2stdvec(histogram);

    polarcontexts_.push_back(ndtmc);
    polarcontext_invkeys_.push_back(histogram);
    polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);
} // makeAndSaveNdtMapCodeAndKeys