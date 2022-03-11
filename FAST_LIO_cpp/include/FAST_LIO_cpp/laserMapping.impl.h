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

#include "laserMapping.h"



#include <iomanip>
#include <omp.h>
#include <math.h>
#include <fstream>
#include <csignal>
//#include <unistd.h>
// #include <Python.h>

#include <Eigen/Core>

// #include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


// #include <FAST_LIO_cpp/messages/Odometry.h>
// #include <FAST_LIO_cpp/messages/Path.h>
// #include <FAST_LIO_cpp/messages/Marker.h>
#include <FAST_LIO_cpp/messages/PointCloud2.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <visualization_msgs/Marker.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <geometry_msgs/Vector3.h>
// #include <livox_ros_driver/CustomMsg.h>

// #include <ros/ros.h>

#include <FAST_LIO_cpp/so3_math.h>
#include <FAST_LIO_cpp/IMU_Processing.h>

// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>

#include <FAST_LIO_cpp/preprocess.h>
#include <FAST_LIO_cpp/macro_defines.h>



namespace fast_lio {

    #define FAST_LIO_LASER_MAPPING_TEMPLATE template<class TCommon, class TPreprocess, class TImuProcess>
    #define FAST_LIO_LASER_MAPPING_CLASS LaserMapping_<TCommon, TPreprocess, TImuProcess>

    FAST_LIO_LASER_MAPPING_TEMPLATE
    inline void FAST_LIO_LASER_MAPPING_CLASS::dump_lio_state_to_log(FILE *fp)  
    {
        V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
        fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
        fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
        fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
        fprintf(fp, "\r\n");  
        fflush(fp);
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    template<class T>
    void FAST_LIO_LASER_MAPPING_CLASS::pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
    {
        V3D p_body(pi[0], pi[1], pi[2]);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    template<typename T>
    void FAST_LIO_LASER_MAPPING_CLASS::set_posestamp(T & out)
    {
        out.pose.position.x = state_point.pos(0);
        out.pose.position.y = state_point.pos(1);
        out.pose.position.z = state_point.pos(2);
        out.pose.orientation.x = geoQuat.x;
        out.pose.orientation.y = geoQuat.y;
        out.pose.orientation.z = geoQuat.z;
        out.pose.orientation.w = geoQuat.w;
    }


    // void SigHandle(int sig)
    // {
    //     flg_exit = true;
    //     printf("catch sig %d", sig);
    //     // ROS_WARN("catch sig %d", sig);
    //     sig_buffer.notify_all();
    // }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::pointBodyToWorld(PointType const * const pi, PointType * const po)
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }


    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
    {
        V3D p_body_lidar(pi->x, pi->y, pi->z);
        V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

        po->x = p_body_imu(0);
        po->y = p_body_imu(1);
        po->z = p_body_imu(2);
        po->intensity = pi->intensity;
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::points_cache_collect()
    {
        PointVector points_history;
        ikdtree.acquire_removed_points(points_history);
        // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::lasermap_fov_segment()
    {
        cub_needrm.clear();
        kdtree_delete_counter = 0;
        kdtree_delete_time = 0.0;    
        pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
        V3D pos_LiD = pos_lid;
        if (!Localmap_Initialized){
            for (int i = 0; i < 3; i++){
                LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
                LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
            }
            Localmap_Initialized = true;
            return;
        }
        float dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; i++){
            dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
        }
        if (!need_move) return;
        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points;
        float mov_dist = std::max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
        for (int i = 0; i < 3; i++){
            tmp_boxpoints = LocalMap_Points;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
        }
        LocalMap_Points = New_LocalMap_Points;

        points_cache_collect();
        double delete_begin = omp_get_wtime();
        if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
        kdtree_delete_time = omp_get_wtime() - delete_begin;
    }

    /*
    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
    {
        mtx_buffer.lock();
        scan_count ++;
        double preprocess_start_time = omp_get_wtime();
        if (msg->header.stamp.toSec() < last_timestamp_lidar)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }

        PointCloudXYZIPtr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(msg->header.stamp.toSec());
        last_timestamp_lidar = msg->header.stamp.toSec();
        s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
    {
        mtx_buffer.lock();
        double preprocess_start_time = omp_get_wtime();
        scan_count ++;
        if (msg->header.stamp.toSec() < last_timestamp_lidar)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }
        last_timestamp_lidar = msg->header.stamp.toSec();
        
        if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
        {
            printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
        }

        if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
        {
            timediff_set_flg = true;
            timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
            printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
        }

        PointCloudXYZIPtr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(last_timestamp_lidar);
        
        s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::imu_cbk(const ImuConstPtr &msg_in) 
    {
        publish_count ++;
        // std::cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<std::endl;
        sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

        if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
        {
            msg->header.stamp = \
            Timestamp().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
        }

        double timestamp = msg->header.stamp.toSec();

        mtx_buffer.lock();

        if (timestamp < last_timestamp_imu)
        {
            ROS_WARN("imu loop back, clear buffer");
            imu_buffer.clear();
        }

        last_timestamp_imu = timestamp;

        w.push_back(msg);
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }
    */


    FAST_LIO_LASER_MAPPING_TEMPLATE
    bool FAST_LIO_LASER_MAPPING_CLASS::sync_packages(MeasureGroup &meas)
    {
        if (lidar_buffer.empty() || imu_buffer.empty()) {
            return false;
        }

        /*** push a lidar scan ***/
        if(!lidar_pushed)
        {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() <= 1) // time too little
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
                // ROS_WARN("Too few input point cloud!\n");
                printf("Too few input point cloud!\n");
            }
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            }
            else
            {
                scan_num ++;
                lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
            }

            meas.lidar_end_time = lidar_end_time;

            lidar_pushed = true;
        }

        if (last_timestamp_imu < lidar_end_time)
        {
            return false;
        }

        /*** push imu data, and pop from imu buffer ***/
        double imu_time = imu_buffer.front()->header.stamp.toSec();
        meas.imu.clear();
        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = imu_buffer.front()->header.stamp.toSec();
            if(imu_time > lidar_end_time) break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;
        return true;
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::map_incremental()
    {
        PointVector PointToAdd;
        PointVector PointNoNeedDownsample;
        PointToAdd.reserve(feats_down_size);
        PointNoNeedDownsample.reserve(feats_down_size);
        for (int i = 0; i < feats_down_size; i++)
        {
            /* transform to world frame */
            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            /* decide if need add to map */
            if (!Nearest_Points[i].empty() && flg_EKF_inited)
            {
                const PointVector &points_near = Nearest_Points[i];
                bool need_add = true;
                BoxPointType Box_of_Point;
                PointType downsample_result, mid_point; 
                mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                float dist  = Common::calc_dist(feats_down_world->points[i],mid_point);
                if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                    PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                    continue;
                }
                for (int readd_i = 0; readd_i < NumMatchPoints::value; readd_i ++)
                {
                    if (points_near.size() < NumMatchPoints::value) break;
                    if (Common::calc_dist(points_near[readd_i], mid_point) < dist)
                    {
                        need_add = false;
                        break;
                    }
                }
                if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
            }
            else
            {
                PointToAdd.push_back(feats_down_world->points[i]);
            }
        }

        double st_time = omp_get_wtime();
        add_point_size = ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false); 
        add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
        kdtree_incremental_time = omp_get_wtime() - st_time;
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::publish_frame_world()
    {
        if(scan_pub_en)
        {
            PointCloudXYZIPtr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
            int size = laserCloudFullRes->points.size();
            PointCloudXYZIPtr laserCloudWorld( \
                            new PointCloudXYZI(size, 1));

            for (int i = 0; i < size; i++)
            {
                RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                    &laserCloudWorld->points[i]);
            }

            // sensor_msgs::PointCloud2 laserCloudmsg;
            // pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
            // laserCloudmsg.header.stamp = Timestamp().fromSec(lidar_end_time);
            // laserCloudmsg.header.frame_id = "camera_init";
            if (cb_publish_frame_world) cb_publish_frame_world(lidar_end_time, laserCloudWorld);
            publish_count -= PUBFRAME_PERIOD;
        }

        /**************** save map ****************/
        /* 1. make sure you have enough memories
        /* 2. noted that pcd save will influence the real-time performences **/
        if (pcd_save_en)
        {
            int size = feats_undistort->points.size();
            PointCloudXYZIPtr laserCloudWorld( \
                            new PointCloudXYZI(size, 1));

            for (int i = 0; i < size; i++)
            {
                RGBpointBodyToWorld(&feats_undistort->points[i], \
                                    &laserCloudWorld->points[i]);
            }
            *pcl_wait_save += *laserCloudWorld;

            static int scan_wait_num = 0;
            scan_wait_num ++;
            if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
            {
                pcd_index ++;
                std::string all_points_dir(std::string(std::string(FAST_LIO_ROOT_DIR) + "PCD/scans_") + std::to_string(pcd_index) + std::string(".pcd"));
                pcl::PCDWriter pcd_writer;
                std::cout << "current scan saved to /PCD/" << all_points_dir << std::endl;
                pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
                pcl_wait_save->clear();
                scan_wait_num = 0;
            }
        }
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::publish_frame_body()
    {
        int size = feats_undistort->points.size();
        PointCloudXYZIPtr laserCloudIMUBody(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                                &laserCloudIMUBody->points[i]);
        }

        // sensor_msgs::PointCloud2 laserCloudmsg;
        // pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
        // laserCloudmsg.header.stamp = Timestamp().fromSec(lidar_end_time);
        // laserCloudmsg.header.frame_id = "body";
        if (cb_publish_frame_body) cb_publish_frame_body(lidar_end_time, laserCloudIMUBody);
        publish_count -= PUBFRAME_PERIOD;
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::publish_effect_world()
    {
        PointCloudXYZIPtr laserCloudWorld( \
                        new PointCloudXYZI(effct_feat_num, 1));
        for (int i = 0; i < effct_feat_num; i++)
        {
            RGBpointBodyToWorld(&laserCloudOri->points[i], \
                                &laserCloudWorld->points[i]);
        }
        // sensor_msgs::PointCloud2 laserCloudFullRes3;
        // pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
        // laserCloudFullRes3.header.stamp = Timestamp().fromSec(lidar_end_time);
        // laserCloudFullRes3.header.frame_id = "camera_init";
        if (cb_publish_effect_world) cb_publish_effect_world(lidar_end_time, laserCloudWorld);
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::publish_map()
    {
        // sensor_msgs::PointCloud2 laserCloudMap;
        // pcl::toROSMsg(*featsFromMap, laserCloudMap);
        // laserCloudMap.header.stamp = Timestamp().fromSec(lidar_end_time);
        // laserCloudMap.header.frame_id = "camera_init";
        if (cb_publish_map) cb_publish_map(lidar_end_time, featsFromMap);
    }


    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::publish_odometry()
    {
        odomAftMapped.header.frame_id = "camera_init";
        odomAftMapped.child_frame_id = "body";
        odomAftMapped.header.stamp = Timestamp().fromSec(lidar_end_time);// Timestamp().fromSec(lidar_end_time);
        set_posestamp(odomAftMapped.pose);
        if (cb_publish_odometry) cb_publish_odometry(lidar_end_time, odomAftMapped);
        auto P = kf.get_P();
        for (int i = 0; i < 6; i ++)
        {
            int k = i < 3 ? i + 3 : i - 3;
            odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
            odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
            odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
            odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
            odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
            odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
        }

        // static tf::TransformBroadcaster br;
        // tf::Transform                   transform;
        // tf::Quaternion                  q;
        // transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
        //                                 odomAftMapped.pose.pose.position.y, \
        //                                 odomAftMapped.pose.pose.position.z));
        // q.setW(odomAftMapped.pose.pose.orientation.w);
        // q.setX(odomAftMapped.pose.pose.orientation.x);
        // q.setY(odomAftMapped.pose.pose.orientation.y);
        // q.setZ(odomAftMapped.pose.pose.orientation.z);
        // transform.setRotation( q );
        // br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::publish_path()
    {
        set_posestamp(msg_body_pose);
        msg_body_pose.header.stamp = Timestamp().fromSec(lidar_end_time);
        msg_body_pose.header.frame_id = "camera_init";

        /*** if path is too large, the rvis will crash ***/
        static int jjj = 0;
        jjj++;
        if (jjj % 10 == 0) 
        {
            path.poses.push_back(msg_body_pose);
            if (cb_publish_path) cb_publish_path(lidar_end_time, path);
        }
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
    {
        double match_start = omp_get_wtime();
        laserCloudOri->clear(); 
        corr_normvect->clear(); 
        total_residual = 0.0; 

        /** closest surface search and residual computation **/
        #ifdef MP_EN
            omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
        #endif
        for (int i = 0; i < feats_down_size; i++)
        {
            PointType &point_body  = feats_down_body->points[i]; 
            PointType &point_world = feats_down_world->points[i]; 

            /* transform to world frame */
            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_world.intensity = point_body.intensity;

            std::vector<float> pointSearchSqDis(NumMatchPoints::value);

            auto &points_near = Nearest_Points[i];

            if (ekfom_data.converge)
            {
                /** Find the closest surfaces in the map **/
                ikdtree.Nearest_Search(point_world, NumMatchPoints::value, points_near, pointSearchSqDis);
                point_selected_surf[i] = points_near.size() < NumMatchPoints::value ? false : pointSearchSqDis[NumMatchPoints::value - 1] > 5 ? false : true;
            }

            if (!point_selected_surf[i]) continue;

            Common::VF<4> pabcd;
            point_selected_surf[i] = false;
            if (Common::esti_plane(pabcd, points_near, 0.1f))
            {
                float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                if (s > 0.9)
                {
                    point_selected_surf[i] = true;
                    normvec->points[i].x = pabcd(0);
                    normvec->points[i].y = pabcd(1);
                    normvec->points[i].z = pabcd(2);
                    normvec->points[i].intensity = pd2;
                    res_last[i] = abs(pd2);
                }
            }
        }
        
        effct_feat_num = 0;

        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_selected_surf[i])
            {
                laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];
                total_residual += res_last[i];
                effct_feat_num ++;
            }
        }

        if (effct_feat_num < 1)
        {
            ekfom_data.valid = false;
            printf("No Effective Points! \n");
            // ROS_WARN("No Effective Points! \n");
            return;
        }

        res_mean_last = total_residual / effct_feat_num;
        match_time  += omp_get_wtime() - match_start;
        double solve_start_  = omp_get_wtime();
        
        /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
        ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12); //23
        ekfom_data.h.resize(effct_feat_num);

        for (int i = 0; i < effct_feat_num; i++)
        {
            const PointType &laser_p  = laserCloudOri->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << FAST_LIO_SKEW_SYM_MATRIX(point_this_be);
            V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat << FAST_LIO_SKEW_SYM_MATRIX(point_this);

            /*** get the normal vector of closest surface/corner ***/
            const PointType &norm_p = corr_normvect->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            /*** calculate the Measuremnt Jacobian matrix H ***/
            V3D C(s.rot.conjugate() *norm_vec);
            V3D A(point_crossmat * C);
            if (extrinsic_est_en)
            {
                V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
                ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, FAST_LIO_VEC_FROM_ARRAY(A), FAST_LIO_VEC_FROM_ARRAY(B), FAST_LIO_VEC_FROM_ARRAY(C);
            }
            else
            {
                ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, FAST_LIO_VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }

            /*** Measuremnt: distance to the closest surface/corner ***/
            ekfom_data.h(i) = -norm_p.intensity;
        }
        solve_time += omp_get_wtime() - solve_start_;
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::setup(const Parameters& params)
    {
        path_en                 = params.publish_path_en;
        scan_pub_en             = params.publish_scan_publish_en;
        dense_pub_en            = params.publish_dense_publish_en;
        scan_body_pub_en        = params.publish_scan_bodyframe_pub_en;
        NUM_MAX_ITERATIONS      = params.max_iteration;
        map_file_path           = params.map_file_path;
        time_sync_en            = params.common_time_sync_en;
        filter_size_corner_min  = params.filter_size_corner;
        filter_size_surf_min    = params.filter_size_surf;
        filter_size_map_min     = params.filter_size_map;
        cube_len                = params.cube_side_length;
        DET_RANGE               = params.mapping_det_range;
        fov_deg                 = params.mapping_fov_degree;
        gyr_cov                 = params.mapping_gyr_cov;
        acc_cov                 = params.mapping_acc_cov;
        b_gyr_cov               = params.mapping_b_gyr_cov;
        b_acc_cov               = params.mapping_b_acc_cov;
        p_pre->blind            = params.preprocess_blind;
        p_pre->lidar_type       = params.preprocess_lidar_type;
        p_pre->N_SCANS          = params.preprocess_scan_line;
        p_pre->SCAN_RATE        = params.preprocess_scan_rate;
        p_pre->point_filter_num = params.point_filter_num;
        p_pre->feature_enabled  = params.feature_extract_enable;
        runtime_pos_log         = params.runtime_pos_log_enable;
        extrinsic_est_en        = params.mapping_extrinsic_est_en;
        pcd_save_en             = params.pcd_save_pcd_save_en;
        pcd_save_interval       = params.pcd_save_interval;
        extrinT                 = params.mapping_extrinsic_T;
        extrinR                 = params.mapping_extrinsic_R;

        std::cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<std::endl;

        path.header.stamp    = Timestamp::now();
        path.header.frame_id ="camera_init";

        /*** variables definition ***/
        
        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU << FAST_LIO_VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << FAST_LIO_MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        double epsi[23] = {0.001};
        std::fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(
            get_f, df_dx, df_dw, 
            [this](auto &s, auto &ekfom_data){ this->h_share_model(s, ekfom_data); }, 
            NUM_MAX_ITERATIONS, epsi
        );

        /*** debug record ***/
        std::string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");

        fout_pre.open(FAST_LIO_DEBUG_FILE_DIR("mat_pre.txt"),std::ios::out);
        fout_out.open(FAST_LIO_DEBUG_FILE_DIR("mat_out.txt"),std::ios::out);
        fout_dbg.open(FAST_LIO_DEBUG_FILE_DIR("dbg.txt"),std::ios::out);
        if (fout_pre && fout_out)
            std::cout << "~~~~"<< FAST_LIO_ROOT_DIR<<" file opened" << std::endl;
        else
            std::cout << "~~~~"<< FAST_LIO_ROOT_DIR<<" doesn't exist" << std::endl;


    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::update()
    {
        if(!sync_packages(Measures)) return;

        if (flg_first_scan)
        {
            first_lidar_time = Measures.lidar_beg_time;
            p_imu->first_lidar_time = first_lidar_time;
            flg_first_scan = false;
            return;
        }

        double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

        match_time = 0;
        kdtree_search_time = 0.0;
        solve_time = 0;
        solve_const_H_time = 0;
        svd_time   = 0;
        t0 = omp_get_wtime();

        p_imu->Process(Measures, kf, feats_undistort);
        state_point = kf.get_x();

        // ambiguous operator*, use operator that performs rotation of row-vector matrix 'offset_T_L_I'
        pos_lid = state_point.pos + state_point.rot * static_cast<typename vect3::base>(state_point.offset_T_L_I);

        if (feats_undistort->empty() || (feats_undistort == NULL))
        {
            printf("No point, skip this scan!\n");
            // ROS_WARN("No point, skip this scan!\n");
            return;
        }

        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                        false : true;
        /*** Segment the map in lidar FOV ***/
        lasermap_fov_segment();

        /*** downsample the feature points in a scan ***/
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        t1 = omp_get_wtime();
        feats_down_size = feats_down_body->points.size();
        /*** initialize the map kdtree ***/
        if(ikdtree.Root_Node == nullptr)
        {
            if(feats_down_size > 5)
            {
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);
                for(int i = 0; i < feats_down_size; i++)
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                ikdtree.Build(feats_down_world->points);
            }
            return;
        }
        int featsFromMapNum = ikdtree.validnum();
        kdtree_size_st = ikdtree.size();
        
        // std::cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<std::endl;

        /*** ICP and iterated Kalman filter update ***/
        if (feats_down_size < 5)
        {
            printf("No point, skip this scan!\n");
            // printf("No point, skip this scan!\n");
            printf("No point, skip this scan!\n");
            // ROS_WARN("No point, skip this scan!\n");
            return;
        }
        
        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);

        V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
        fout_pre<<std::setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
        <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< std::endl;

        if(0) // If you need to see map point, change to "if(1)"
        {
            PointVector ().swap(ikdtree.PCL_Storage);
            ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
            featsFromMap->clear();
            featsFromMap->points = ikdtree.PCL_Storage;
        }

        pointSearchInd_surf.resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);
        int  rematch_num = 0;
        bool nearest_search_en = true; //

        t2 = omp_get_wtime();
        
        /*** iterated state estimation ***/
        double t_update_start = omp_get_wtime();
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        state_point = kf.get_x();
        euler_cur = SO3ToEuler(state_point.rot);
        // ambiguous operator*, use operator that performs rotation of row-vector matrix 'offset_T_L_I'
        pos_lid = state_point.pos + state_point.rot * static_cast<typename vect3::base>(state_point.offset_T_L_I);
        geoQuat.x = state_point.rot.coeffs()[0];
        geoQuat.y = state_point.rot.coeffs()[1];
        geoQuat.z = state_point.rot.coeffs()[2];
        geoQuat.w = state_point.rot.coeffs()[3];

        double t_update_end = omp_get_wtime();

        /******* Publish odometry *******/
        publish_odometry();

        /*** add the feature points to map kdtree ***/
        t3 = omp_get_wtime();
        map_incremental();
        t5 = omp_get_wtime();
        
        /******* Publish points *******/
        if (path_en)                         publish_path();
        if (scan_pub_en || pcd_save_en)      publish_frame_world();
        if (scan_pub_en && scan_body_pub_en) publish_frame_body();
        // publish_effect_world();
        // publish_map();

        /*** Debug variables ***/
        if (runtime_pos_log)
        {
            frame_num ++;
            kdtree_size_end = ikdtree.size();
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
            aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
            aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
            aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
            aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
            aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
            T1[time_log_counter] = Measures.lidar_beg_time;
            s_plot[time_log_counter] = t5 - t0;
            s_plot2[time_log_counter] = feats_undistort->points.size();
            s_plot3[time_log_counter] = kdtree_incremental_time;
            s_plot4[time_log_counter] = kdtree_search_time;
            s_plot5[time_log_counter] = kdtree_delete_counter;
            s_plot6[time_log_counter] = kdtree_delete_time;
            s_plot7[time_log_counter] = kdtree_size_st;
            s_plot8[time_log_counter] = kdtree_size_end;
            s_plot9[time_log_counter] = aver_time_consu;
            s_plot10[time_log_counter] = add_point_size;
            time_log_counter ++;
            printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
            ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_out << std::setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<std::endl;
            dump_lio_state_to_log(fp);
        }

    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::finish()
    {
        /**************** save map ****************/
        /* 1. make sure you have enough memories
        /* 2. pcd save will largely influence the real-time performences **/
        if (pcl_wait_save->size() > 0 && pcd_save_en)
        {
            std::string file_name = std::string("scans.pcd");
            std::string all_points_dir(std::string(std::string(FAST_LIO_ROOT_DIR) + "PCD/") + file_name);
            pcl::PCDWriter pcd_writer;
            std::cout << "current scan saved to /PCD/" << file_name<<std::endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
        }

        fout_out.close();
        fout_pre.close();

        if (runtime_pos_log)
        {
            std::vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
            FILE *fp2;
            std::string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
            fp2 = fopen(log_dir.c_str(),"w");
            fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
            for (int i = 0;i<time_log_counter; i++){
                fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
                t.push_back(T1[i]);
                s_vec.push_back(s_plot9[i]);
                s_vec2.push_back(s_plot3[i] + s_plot6[i]);
                s_vec3.push_back(s_plot4[i]);
                s_vec5.push_back(s_plot[i]);
            }
            fclose(fp2);
        }

    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::observeImu(double time, const Imu& msg_in)
    {
        publish_count ++;
        // std::cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<std::endl;
        ImuPtr msg(new Imu(msg_in));

        if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
        {
            msg->header.stamp = Timestamp().fromSec(timediff_lidar_wrt_imu + time);
        }

        double timestamp = msg->header.stamp.toSec();

        mtx_buffer.lock();

        if (timestamp < last_timestamp_imu)
        {
            printf("imu loop back, clear buffer");
            // printf("imu loop back, clear buffer");
            ROS_WARN("imu loop back, clear buffer");
            imu_buffer.clear();
        }

        last_timestamp_imu = timestamp;

        imu_buffer.push_back(msg);
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::observeLidar(double time, const PointCloudLivox& msg)
    {
        mtx_buffer.lock();
        double preprocess_start_time = omp_get_wtime();
        scan_count ++;
        if (time < last_timestamp_lidar)
        {
            // ROS_ERROR("lidar loop back, clear buffer");
            printf("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }
        last_timestamp_lidar = time;
        
        if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
        {
            printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
        }

        if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
        {
            timediff_set_flg = true;
            timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
            printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
        }

        PointCloudXYZIPtr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(last_timestamp_lidar);
        
        s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::observeLidar(double time, const PointCloudOuster& msg)
    {
        mtx_buffer.lock();
        scan_count ++;
        double preprocess_start_time = omp_get_wtime();
        if (time < last_timestamp_lidar)
        {
            // ROS_ERROR("lidar loop back, clear buffer");
            printf("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }

        PointCloudXYZIPtr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(time);
        last_timestamp_lidar = time;
        s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    FAST_LIO_LASER_MAPPING_TEMPLATE
    void FAST_LIO_LASER_MAPPING_CLASS::observeLidar(double time, const PointCloudVelodyne& msg)
    {
        mtx_buffer.lock();
        scan_count ++;
        double preprocess_start_time = omp_get_wtime();
        if (time < last_timestamp_lidar)
        {
            // ROS_ERROR("lidar loop back, clear buffer");
            printf("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }

        PointCloudXYZIPtr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(time);
        last_timestamp_lidar = time;
        s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }



    #undef FAST_LIO_LASER_MAPPING_TEMPLATE
    #undef FAST_LIO_LASER_MAPPING_CLASS

} // namespace fast_lio
