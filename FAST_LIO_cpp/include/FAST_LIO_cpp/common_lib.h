#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <vector>
#include <string>
#include <deque>
#include <type_traits>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <FAST_LIO_cpp/so3_math.h>

#include <FAST_LIO_cpp/messages/Pose6D.h>
#include <FAST_LIO_cpp/messages/Imu.h>
#include <FAST_LIO_cpp/messages/Quaternion.h>
#include <FAST_LIO_cpp/messages/Odometry.h>
#include <FAST_LIO_cpp/messages/Path.h>
#include <FAST_LIO_cpp/messages/PoseStamped.h>

#include <FAST_LIO_cpp/messages/lidars/PointCloudLivox.h>
#include <FAST_LIO_cpp/messages/lidars/PointCloudOuster.h>
#include <FAST_LIO_cpp/messages/lidars/PointCloudVelodyne.h>

// #include <tf/transform_broadcaster.h>
// #include <eigen_conversions/eigen_msg.h>

namespace fast_lio {

template<
    class TPose6D,             // fast_lio::Pose6D
    class TImu,                // fast_lio::Imu
    class TQuaternion,         // fast_lio::Quaternion
    class TOdometry,           // fast_lio::Odometry
    class TPath,               // fast_lio::Path
    class TPoseStamped,        // fast_lio::PoseStamped
    class TPointCloudLivox,    // fast_lio::PointCloudLivox
    class TPointCloudOuster,   // fast_lio::PointCloudOuster
    class TPointCloudVelodyne, // fast_lio::PointCloudVelodyne
    class TPointType,          // pcl::PointXYZINormal
    class TPointCloudXYZI,     // pcl::PointCloud<PointType>
    unsigned int TNumMatchPoints = 5
>
struct Common_
{
    using Pose6D = TPose6D;
    using Imu = TImu;

    using Quaternion         = TQuaternion; // TODO add to template
    using Odometry           = TOdometry; // TODO add to template
    using Path               = TPath; // TODO add to template
    using PoseStamped        = TPoseStamped; // TODO add to template
    using PointCloudLivox    = TPointCloudLivox; // TODO add to template
    using PointCloudOuster   = TPointCloudOuster; // TODO add to template
    using PointCloudVelodyne = TPointCloudVelodyne; // TODO add to template

    using PointType = TPointType;
    using PointCloudXYZI = TPointCloudXYZI;

    using NumMatchPoints = std::integral_constant<unsigned int, TNumMatchPoints>;

    using PointCloudXYZIPtr = typename PointCloudXYZI::Ptr;
    using ImuPtr = typename Imu::Ptr;
    using ImuConstPtr = typename Imu::ConstPtr;

    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    using V3D = Eigen::Vector3d;
    using M3D = Eigen::Matrix3d;
    using V3F = Eigen::Vector3f;
    using M3F = Eigen::Matrix3f;

    template<unsigned int Rows, unsigned int Cols>
    using MD = Eigen::Matrix<double, Rows, Cols>;
    
    template<unsigned int Rows>
    using VD = Eigen::Matrix<double, Rows, 1>;
    
    template<unsigned int A, unsigned int B>
    using MF = Eigen::Matrix<float, A, B>;

    template<unsigned int Rows>
    using VF = Eigen::Matrix<float, Rows, 1>;

    static const M3D Eye3d;
    static const M3F Eye3f;
    static const V3D Zero3d;
    static const V3F Zero3f;

    struct MeasureGroup     // Lidar data and imu dates for the current process
    {
        MeasureGroup()
        {
            lidar_beg_time = 0.0;
            this->lidar.reset(new PointCloudXYZI());
        };
        double lidar_beg_time;
        double lidar_end_time;
        PointCloudXYZIPtr lidar;
        std::deque<ImuConstPtr> imu; 
    };

    // unused
    struct StatesGroup
    {
        using DimState = std::integral_constant<unsigned int, 18>; // Dimension of states (Let Dim(SO(3)) = 3)
        using DimProcN = std::integral_constant<unsigned int, 12>; // Dimension of process noise (Let Dim(SO(3)) = 3), unused

        StatesGroup(double initial_covariance = 1.0);
        StatesGroup(const StatesGroup& b);

        StatesGroup& operator=(const StatesGroup& b);
        StatesGroup operator+(const Eigen::Matrix<double, DimState, 1> &state_add);
        StatesGroup& operator+=(const Eigen::Matrix<double, DimState, 1> &state_add);
        Eigen::Matrix<double, DimState, 1> operator-(const StatesGroup& b);

        void resetpose();

        M3D rot_end;      // the estimated attitude (rotation matrix) at the end lidar point
        V3D pos_end;      // the estimated position at the end lidar point (world frame)
        V3D vel_end;      // the estimated velocity at the end lidar point (world frame)
        V3D bias_g;       // gyroscope bias
        V3D bias_a;       // accelerator bias
        V3D gravity;      // the estimated gravity acceleration
        Eigen::Matrix<double, DimState, DimState>  cov;     // states covariance
    };


    template<typename T>
    static auto set_pose6d(
        const double t, 
        const Eigen::Matrix<T, 3, 1> &a, 
        const Eigen::Matrix<T, 3, 1> &g,
        const Eigen::Matrix<T, 3, 1> &v, 
        const Eigen::Matrix<T, 3, 1> &p, 
        const Eigen::Matrix<T, 3, 3> &R
    )
    {
        Pose6D rot_kp;
        rot_kp.offset_time = t;
        for (int i = 0; i < 3; i++)
        {
            rot_kp.acc[i] = a(i);
            rot_kp.gyr[i] = g(i);
            rot_kp.vel[i] = v(i);
            rot_kp.pos[i] = p(i);
            for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j);
        }
        return std::move(rot_kp);
    }

    /* comment
    plane equation: Ax + By + Cz + D = 0
    convert to: A/D*x + B/D*y + C/D*z = -1
    solve: A0*x0 = b0
    where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
    normvec:  normalized x0
    */
    template<typename T>
    static bool esti_normvector(
        Eigen::Matrix<T, 3, 1> &normvec, 
        const PointVector &point, 
        const T &threshold, 
        const int &point_num)
    {
        MatrixXf A(point_num, 3);
        MatrixXf b(point_num, 1);
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < point_num; j++)
        {
            A(j,0) = point[j].x;
            A(j,1) = point[j].y;
            A(j,2) = point[j].z;
        }
        normvec = A.colPivHouseholderQr().solve(b);
        
        for (int j = 0; j < point_num; j++)
        {
            if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
            {
                return false;
            }
        }

        normvec.normalize();
        return true;
    }

    // todo: rename to calc_dist2 as it returns squared distance
    static float calc_dist(PointType p1, PointType p2){
        float d2 = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
        return d2;
    }

    template<typename T>
    static bool esti_plane(
        Eigen::Matrix<T, 4, 1> &pca_result, 
        const PointVector &point, 
        const T &threshold
    )
    {
        Eigen::Matrix<T, NumMatchPoints::value, 3> A;
        Eigen::Matrix<T, NumMatchPoints::value, 1> b;
        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < NumMatchPoints::value; j++)
        {
            A(j,0) = point[j].x;
            A(j,1) = point[j].y;
            A(j,2) = point[j].z;
        }

        Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

        T n = normvec.norm();
        pca_result(0) = normvec(0) / n;
        pca_result(1) = normvec(1) / n;
        pca_result(2) = normvec(2) / n;
        pca_result(3) = 1.0 / n;

        for (int j = 0; j < NumMatchPoints::value; j++)
        {
            if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
            {
                return false;
            }
        }
        return true;
    }

};

#define FAST_LIO_COMMON_TEMPLATE template<class TPose6D, class TImu, class TQuaternion, class TOdometry, class TPath, class TPoseStamped, class TPointCloudLivox, class TPointCloudOuster, class TPointCloudVelodyne, class TPointType, class TPointCloudXYZI, unsigned int TNumMatchPoints>
#define FAST_LIO_COMMON_CLASS Common_<TPose6D, TImu, TQuaternion, TOdometry, TPath, TPoseStamped, TPointCloudLivox, TPointCloudOuster, TPointCloudVelodyne, TPointType, TPointCloudXYZI, TNumMatchPoints>
FAST_LIO_COMMON_TEMPLATE const FAST_LIO_COMMON_CLASS::M3D FAST_LIO_COMMON_CLASS::Eye3d(M3D::Identity());
FAST_LIO_COMMON_TEMPLATE const FAST_LIO_COMMON_CLASS::M3F FAST_LIO_COMMON_CLASS::Eye3d(M3F::Identity());
FAST_LIO_COMMON_TEMPLATE const FAST_LIO_COMMON_CLASS::V3D FAST_LIO_COMMON_CLASS::Zero3d(0,0,0);
FAST_LIO_COMMON_TEMPLATE const FAST_LIO_COMMON_CLASS::V3F FAST_LIO_COMMON_CLASS::Zero3f(0,0,0);
#undef FAST_LIO_COMMON_CLASS
#undef FAST_LIO_COMMON_TEMPLATE

using CommonCpp = Common_<
    fast_lio::Pose6D,
    fast_lio::Imu,
    fast_lio::Quaternion,
    fast_lio::Odometry,
    fast_lio::Path,
    fast_lio::PoseStamped,
    fast_lio::PointCloudLivox,
    fast_lio::PointCloudOuster,
    fast_lio::PointCloudVelodyne,
    pcl::PointXYZINormal,
    pcl::PointCloud<pcl::PointXYZINormal>,
    5
>;

#define PI_M (3.14159265358)

template<typename T>
T rad2deg(T radians)
{
  return radians * 180.0 / PI_M;
}

template<typename T>
T deg2rad(T degrees)
{
  return degrees * PI_M / 180.0;
}

} // namespace fast_lio


#endif