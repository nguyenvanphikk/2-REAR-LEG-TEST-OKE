#ifndef _RobotState
#define _RobotState

#include <eigen3/Eigen/Dense>
#include "common_types.h"

using Eigen::Matrix;
using Eigen::Quaternionf;

#include "common_types.h"
class RobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w;
        Matrix<fpt,3,4> r_feet;
        Matrix<fpt,3,3> R;
        Matrix<fpt,3,3> R_yaw;
        Matrix<fpt,3,3> I_body;
        Quaternionf q;
        fpt yaw;
        // Tong khoi luong robot MIT_3HP theo URDF, don vi kg. Dense MPC dung
        // gia tri nay trong quan he gia toc tinh tien = tong luc chan / mass.
        fpt m = 10.865863917f;
        //fpt m = 50.236; //DH
    //private:
};
#endif
