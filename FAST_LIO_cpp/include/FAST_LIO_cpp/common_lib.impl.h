#pragma once
#include <FAST_LIO_cpp/common_lib.h>

namespace fast_lio {

#define FAST_LIO_COMMON_TEMPLATE template<class TPose6D, class TImu, class TQuaternion, class TOdometry, class TPath, class TPoseStamped, class TPointCloudLivox, class TPointCloudOuster, class TPointCloudVelodyne, class TPointType, class TPointCloudXYZI, unsigned int TNumMatchPoints>
#define FAST_LIO_COMMON_CLASS Common_<TPose6D, TImu, TQuaternion, TOdometry, TPath, TPoseStamped, TPointCloudLivox, TPointCloudOuster, TPointCloudVelodyne, TPointType, TPointCloudXYZI, TNumMatchPoints>

    FAST_LIO_COMMON_TEMPLATE
    FAST_LIO_COMMON_CLASS::StatesGroup::StatesGroup(double initial_covariance) {
        this->rot_end = M3D::Identity();
        this->pos_end = Zero3d;
        this->vel_end = Zero3d;
        this->bias_g  = Zero3d;
        this->bias_a  = Zero3d;
        this->gravity = Zero3d;
        this->cov     = MD(DimState::value,DimState::value)::Identity() * initial_covariance;
        this->cov.block<9,9>(9,9) = MD(9,9)::Identity() * 0.00001;
    };

    FAST_LIO_COMMON_TEMPLATE
    FAST_LIO_COMMON_CLASS::StatesGroup::StatesGroup(const StatesGroup& b) {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        this->cov     = b.cov;
    }

    FAST_LIO_COMMON_TEMPLATE
    typename FAST_LIO_COMMON_CLASS::StatesGroup& FAST_LIO_COMMON_CLASS::StatesGroup::operator=(const StatesGroup& b)
    {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        this->cov     = b.cov;
        return *this;
    }

    FAST_LIO_COMMON_TEMPLATE
    typename FAST_LIO_COMMON_CLASS::StatesGroup FAST_LIO_COMMON_CLASS::StatesGroup::operator+(const StateMatrix &state_add)
    {
        StatesGroup a;
        a.rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
        a.pos_end = this->pos_end + state_add.block<3,1>(3,0);
        a.vel_end = this->vel_end + state_add.block<3,1>(6,0);
        a.bias_g  = this->bias_g  + state_add.block<3,1>(9,0);
        a.bias_a  = this->bias_a  + state_add.block<3,1>(12,0);
        a.gravity = this->gravity + state_add.block<3,1>(15,0);
        a.cov     = this->cov;
        return a;
    }

    FAST_LIO_COMMON_TEMPLATE
    typename FAST_LIO_COMMON_CLASS::StatesGroup& FAST_LIO_COMMON_CLASS::StatesGroup::operator+=(const StateMatrix &state_add)
    {
        this->rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
        this->pos_end += state_add.block<3,1>(3,0);
        this->vel_end += state_add.block<3,1>(6,0);
        this->bias_g  += state_add.block<3,1>(9,0);
        this->bias_a  += state_add.block<3,1>(12,0);
        this->gravity += state_add.block<3,1>(15,0);
        return *this;
    }

    FAST_LIO_COMMON_TEMPLATE
    typename FAST_LIO_COMMON_CLASS::StatesGroup::StateMatrix FAST_LIO_COMMON_CLASS::StatesGroup::operator-(const StatesGroup& b)
    {
        StateMatrix a;
        M3D rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3,1>(0,0)  = Log(rotd);
        a.block<3,1>(3,0)  = this->pos_end - b.pos_end;
        a.block<3,1>(6,0)  = this->vel_end - b.vel_end;
        a.block<3,1>(9,0)  = this->bias_g  - b.bias_g;
        a.block<3,1>(12,0) = this->bias_a  - b.bias_a;
        a.block<3,1>(15,0) = this->gravity - b.gravity;
        return a;
    }

    FAST_LIO_COMMON_TEMPLATE
    void FAST_LIO_COMMON_CLASS::StatesGroup::resetpose()
    {
        this->rot_end = M3D::Identity();
        this->pos_end = Zero3d;
        this->vel_end = Zero3d;
    }

#undef FAST_LIO_COMMON_CLASS
#undef FAST_LIO_COMMON_TEMPLATE


} // namespace fast_lio
