#pragma once
// #include <ros/ros.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <livox_ros_driver/CustomMsg.h>

#include <FAST_LIO_cpp/common_lib.h>
#include <FAST_LIO_cpp/messages/lidars/PointCloudLivox.h>
#include <FAST_LIO_cpp/messages/lidars/PointCloudOuster.h>
#include <FAST_LIO_cpp/messages/lidars/PointCloudVelodyne.h>

// TODO: add includes for pcl
// TODO: remove ros dependencies

namespace fast_lio { 

template<class TCommon>
class Preprocess_
{
  public:
  using Common = TCommon;

  using PointCloudLivox    = typename Common::PointCloudLivox;
  using PointCloudOuster   = typename Common::PointCloudOuster;
  using PointCloudVelodyne = typename Common::PointCloudVelodyne;

  using PointType         = typename Common::PointType;
  using PointCloudXYZI    = typename Common::PointCloudXYZI;
  using PointCloudXYZIPtr = typename Common::PointCloudXYZIPtr;

  enum LID_TYPE{AVIA = 1, VELO16, OUST64}; //{1, 2, 3}
  enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
  enum Surround{Prev, Next};
  enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

  struct orgtype
  {
    double range;
    double dista; 
    double angle[2];
    double intersect;
    E_jump edj[2];
    Feature ftype;
    orgtype()
    {
      range = 0;
      edj[Prev] = Nr_nor;
      edj[Next] = Nr_nor;
      ftype = Nor;
      intersect = 2;
    }
  };

//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess_();
  ~Preprocess_();
  
  void process(const PointCloudLivox &msg, PointCloudXYZIPtr &pcl_out);
  void process(const PointCloudOuster &msg, PointCloudXYZIPtr &pcl_out);
  void process(const PointCloudVelodyne &msg, PointCloudXYZIPtr &pcl_out);

  // void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZIPtr &pcl_out);
  // void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZIPtr &pcl_out);
  
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  std::vector<orgtype> typess[128]; //maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE;
  double blind;
  bool feature_enabled, given_offset_time;
  // ros::Publisher pub_full, pub_surf, pub_corn;
    

  private:
  using uint = unsigned int;
  void avia_handler(const PointCloudLivox &msg);
  void oust64_handler(const PointCloudOuster &msg);
  void velodyne_handler(const PointCloudVelodyne &msg);
  void give_feature(PointCloudXYZI &pl, std::vector<orgtype> &types);
  // void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};

using PreprocessCpp = Preprocess_<CommonCpp>;

} // namespace fast_lio

#include "preprocess.impl.h"
