// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <memory>
#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <string>
#include <functional>
#include <cstdio>
#include <fstream>

#include <pcl/filters/voxel_grid.h>

#include <FAST_LIO_cpp/common_lib.h>
#include <FAST_LIO_cpp/preprocess.h>
#include <FAST_LIO_cpp/IMU_Processing.h>
#include <FAST_LIO_cpp/ikd-Tree/ikd_Tree.h>
#include <FAST_LIO_cpp/use-ikfom.hpp>

namespace fast_lio {

    template<
        class TCommon, 
        class TPreprocess = Preprocess_<TCommon>,
        class TImuProcess = ImuProcess_<TCommon>
    >
    struct LaserMapping_
    {
    public: // types
        using Common = TCommon;
        using Preprocess = TPreprocess;
        using ImuProcess = TImuProcess;

        using PointType          = typename Common::PointType;
        using PointVector        = typename Common::PointVector;
        using PointCloudXYZIPtr  = typename Common::PointCloudXYZIPtr;
        using Quaternion         = typename Common::Quaternion;
        using Odometry           = typename Common::Odometry;
        using Path               = typename Common::Path;
        using PoseStamped        = typename Common::PoseStamped;
        using PointLivox         = typename Common::PointLivox;
        using PointOuster        = typename Common::PointOuster;
        using PointVelodyne      = typename Common::PointVelodyne;
        using PointCloudLivox    = typename Common::PointCloudLivox;
        using PointCloudOuster   = typename Common::PointCloudOuster;
        using PointCloudVelodyne = typename Common::PointCloudVelodyne;
        using Imu                = typename Common::Imu;
        using ImuPtr             = typename Common::ImuPtr;
        using ImuConstPtr        = typename Common::ImuConstPtr;

        using V3F  = typename Common::V3F;
        using V3D  = typename Common::V3D;
        using M3D  = typename Common::M3D;

        using MeasureGroup = typename Common::MeasureGroup;

        using CallbackPublishPointCloud = std::function<void(double time, const PointCloudXYZIPtr&)>;
        using CallbackPublishOdometry = std::function<void(double time, const Odometry&)>;
        using CallbackPublishPath = std::function<void(double time, const Path&)>;

    protected: // members
        static constexpr double INIT_TIME = 0.1;
        static constexpr double LASER_POINT_COV = 0.001;
        static constexpr unsigned int MAXN = 720000;
        static constexpr int PUBFRAME_PERIOD = 20;

        /*** Time Log Variables ***/
        double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
        double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
        double match_time = 0, solve_time = 0, solve_const_H_time = 0;
        int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
        bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
        /**************************/

        float res_last[100000] = {0.0};
        float DET_RANGE = 300.0f;
        const float MOV_THRESHOLD = 1.5f;

        std::mutex mtx_buffer;
        std::condition_variable sig_buffer;

        std::string root_dir = ROOT_DIR;
        std::string map_file_path, lid_topic, imu_topic;

        double res_mean_last = 0.05, total_residual = 0.0;
        double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
        double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
        double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
        double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
        int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
        int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
        bool   point_selected_surf[100000] = {0};
        bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
        bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

        std::vector<std::vector<int>>  pointSearchInd_surf; 
        // BoxPointType comes from ikd_Tree.h
        std::vector<BoxPointType> cub_needrm; 
        std::vector<PointVector>  Nearest_Points; 
        std::vector<double>       extrinT = std::vector<double>(3, 0.0);
        std::vector<double>       extrinR = std::vector<double>(9, 0.0);
        std::deque<double>                   time_buffer;
        std::deque<PointCloudXYZIPtr>        lidar_buffer;
        std::deque<ImuConstPtr> imu_buffer;

        PointCloudXYZIPtr featsFromMap     = PointCloudXYZIPtr(new PointCloudXYZI());
        PointCloudXYZIPtr feats_undistort  = PointCloudXYZIPtr(new PointCloudXYZI());
        PointCloudXYZIPtr feats_down_body  = PointCloudXYZIPtr(new PointCloudXYZI());
        PointCloudXYZIPtr feats_down_world = PointCloudXYZIPtr(new PointCloudXYZI());
        PointCloudXYZIPtr normvec          = PointCloudXYZIPtr(new PointCloudXYZI(100000, 1));
        PointCloudXYZIPtr laserCloudOri    = PointCloudXYZIPtr(new PointCloudXYZI(100000, 1));
        PointCloudXYZIPtr corr_normvect    = PointCloudXYZIPtr(new PointCloudXYZI(100000, 1));
        PointCloudXYZIPtr _featsArray;

        pcl::VoxelGrid<PointType> downSizeFilterSurf;
        pcl::VoxelGrid<PointType> downSizeFilterMap;

        // KD_TREE comes from ikd_Tree.h
        KD_TREE<PointType> ikdtree;

        const double LIDAR_SP_LEN = 2.0;
        V3F XAxisPoint_body = V3F(LIDAR_SP_LEN, 0.0, 0.0);
        V3F XAxisPoint_world = V3F(LIDAR_SP_LEN, 0.0, 0.0);
        V3D euler_cur;
        V3D position_last = V3D(Common::Zero3d);
        V3D Lidar_T_wrt_IMU = V3D(Common::Zero3d);
        M3D Lidar_R_wrt_IMU = M3D(Common::Eye3d);

        /*** EKF inputs and output ***/
        MeasureGroup Measures;
        esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
        state_ikfom state_point;
        vect3 pos_lid;

        Path path;
        Odometry odomAftMapped;
        Quaternion geoQuat;
        PoseStamped msg_body_pose;

        std::shared_ptr<Preprocess> p_pre = std::shared_ptr<Preprocess>(new Preprocess());
        std::shared_ptr<ImuProcess> p_imu = std::shared_ptr<ImuProcess>(new ImuProcess());

        BoxPointType LocalMap_Points;
        bool Localmap_Initialized = false;

        double timediff_lidar_wrt_imu = 0.0;
        bool   timediff_set_flg = false;

        double lidar_mean_scantime = 0.0;
        int    scan_num = 0;

        int process_increments = 0;

        // (unused) PointCloudXYZIPtr pcl_wait_pub(new PointCloudXYZI(500000, 1)); 
        PointCloudXYZIPtr pcl_wait_save = PointCloudXYZIPtr(new PointCloudXYZI());

        int effect_feat_num = 0, frame_num = 0;
        double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        bool flg_EKF_converged, EKF_stop_flg = 0;


        FILE *fp;

        std::ofstream fout_pre, fout_out, fout_dbg;

    protected: // methods
        void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s);
        void pointBodyToWorld(PointType const * const pi, PointType * const po);
        inline void dump_lio_state_to_log(FILE *fp);
        template<class T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
        void RGBpointBodyToWorld(PointType const * const pi, PointType * const po);
        void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po);

        void points_cache_collect();
        void lasermap_fov_segment();

        // void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
        // void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg);
        // void imu_cbk(const ImuConstPtr &msg_in);

        bool sync_packages(MeasureGroup &meas);
        void map_incremental();
        
        template<typename T> void set_posestamp(T & out);

        void publish_frame_world();
        void publish_frame_body();
        void publish_effect_world();
        void publish_map();

        void publish_odometry();
        void publish_path();

        void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

    public:
        struct Parameters
        {
            bool         publish_path_en               = true            ;
            bool         publish_scan_publish_en       = true            ;
            bool         publish_dense_publish_en      = true            ;
            bool         publish_scan_bodyframe_pub_en = true            ;
            int          max_iteration                 = 4               ;
            std::string  map_file_path                 = ""              ;
            bool         common_time_sync_en           =  false          ;
            double       filter_size_corner            = 0.5             ;
            double       filter_size_surf              = 0.5             ;
            double       filter_size_map               = 0.5             ;
            double       cube_side_length              = 200             ;
            float        mapping_det_range             = 300.f           ;
            double       mapping_fov_degree            = 180             ;
            double       mapping_gyr_cov               = 0.1             ;
            double       mapping_acc_cov               = 0.1             ;
            double       mapping_b_gyr_cov             = 0.0001          ;
            double       mapping_b_acc_cov             = 0.0001          ;
            double       preprocess_blind              = 0.01            ;
            int          preprocess_lidar_type         = 1 /* AVIA */    ;
            int          preprocess_scan_line          = 16              ;
            int          preprocess_scan_rate          = 10              ;
            int          point_filter_num              = 2               ;
            bool         feature_extract_enable        = false           ;
            bool         runtime_pos_log_enable        = 0               ;
            bool         mapping_extrinsic_est_en      = true            ;
            bool         pcd_save_pcd_save_en          = false           ;
            int          pcd_save_interval             = -1              ;
            std::vector<double> mapping_extrinsic_T    = std::vector<double>();
            std::vector<double> mapping_extrinsic_R    = std::vector<double>();
        };
        void setup(const Parameters& params);

        void update();
        
        void finish();

        void observeImu(double time, const Imu& msg);
        void observeLidar(double time, const PointCloudLivox& msg);
        void observeLidar(double time, const PointCloudOuster& msg);
        void observeLidar(double time, const PointCloudVelodyne& msg);

        CallbackPublishPointCloud  cb_publish_frame_world;
        CallbackPublishPointCloud  cb_publish_frame_body;
        CallbackPublishPointCloud  cb_publish_effect_world;
        CallbackPublishPointCloud  cb_publish_map;
        CallbackPublishOdometry    cb_publish_odometry;
        CallbackPublishPath        cb_publish_path;

    };

    using LaserMappingCpp = LaserMapping_<CommonCpp, PreprocessCpp, ImuProcessCpp>;

} // namespace fast_lio

#include "laserMapping.impl.h"
