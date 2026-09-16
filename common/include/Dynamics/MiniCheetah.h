/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH;

  // Khoi luong than da gop base_link va imu_link (khop IMU la khop fixed).
  // Gia tri gop duoc tinh lai COM va quan tinh bang dinh ly truc song song.
  cheetah._bodyMass = 4.62826296379729;
  // Khoang cach giua cac truc ab/ad trong URDF MIT_3HP. Day la kich thuoc dinh
  // vi truc khop, khong phai kich thuoc bao ngoai cua mesh than.
  cheetah._bodyLength = 0.1513 * 2;  // khoang ab/ad truoc-sau: 0.3026 m
  cheetah._bodyWidth = 0.0500 * 2;   // khoang ab/ad trai-phai: 0.1000 m
  cheetah._bodyHeight = 0.05 * 2;
  // Ti so truyen tu rotor motor den truc khop. Knee gom hop so motor 6:1 va
  // bo truyen dai ngoai 1.5:1, nen tong ti so truyen la 9:1.
  cheetah._abadGearRatio = 6;
  cheetah._hipGearRatio = 6;
  cheetah._kneeGearRatio = 6 * 1.5;
  // Cac gia tri vo huong giu tuong thich voi controller cu. Vector tinh tien
  // day du giua cac khop duoc luu trong cac bien Vec3 o phia duoi.
  cheetah._abadLinkLength = 0.0133;  // do lech ngang tu ab/ad den hip
  cheetah._hipLinkLength = 0.2100;   // thanh phan doc tu hip den knee
  //cheetah._kneeLinkLength = 0.175;
  //cheetah._maxLegLength = 0.384;
  // Do lech ngang hip-knee 0.072 m da nam trong _kneeLocation, khong duoc cong
  // them lan nua tai diem tiep xuc ban chan.
  cheetah._kneeLinkY_offset = 0.0;
  //cheetah._kneeLinkLength = 0.20;
  cheetah._kneeLinkLength = 0.1855069879;  // khoang knee-foot trong URDF
  // Khoang cach ab/ad-foot tai tu the xuat q=(0,0,0). Can kiem tra lai tren
  // toan mien goc khop truoc khi dung lam gioi han an toan cung.
  cheetah._maxLegLength = 0.408458;


  cheetah._motorTauMax = 3.f;
  cheetah._batteryV = 24;
  cheetah._motorKT = .05;  // this is flux linkage * pole pairs
  cheetah._motorR = 0.173;
  cheetah._jointDamping = .01;
  cheetah._jointDryFriction = .2;
  //cheetah._jointDamping = .0;
  //cheetah._jointDryFriction = .0;


  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  // Khoi hip-roll/abad cua chan FL trong URDF. Source dung mot bo thong so
  // chuan cho chan trai va doi xung sang chan phai khi build model.
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia <<
      T(0.00040138), T(-0.00000010995), T(0.000000119),
      T(-0.00000010995), T(0.00065415), T(-0.000000036239),
      T(0.000000119), T(-0.000000036239), T(0.00039475);
  Vec3<T> abadCOM(T(0.054913), T(-0.0054134), T(-0.0000079452));
  SpatialInertia<T> abadInertia(T(0.65102), abadCOM,
                                abadRotationalInertia);

  // Khoi thigh/hip cua chan FL trong URDF. Ma tran quan tinh co don vi kg.m^2
  // va duoc khai bao quanh COM cua link, khong phai quanh truc khop.
  Mat3<T> hipRotationalInertia;
  hipRotationalInertia <<
      T(0.000565549775058476), T(-0.000000380617595114868),
      T(0.0000077413062234626), T(-0.000000380617595114868),
      T(0.000900281116356473), T(0.000000656602301302992),
      T(0.0000077413062234626), T(0.000000656602301302992),
      T(0.000490044198301144);
  Vec3<T> hipCOM(T(0.000227780216045637), T(0.039582075948556),
                 T(-0.00582440278066959));
  SpatialInertia<T> hipInertia(T(0.847592785507126), hipCOM,
                               hipRotationalInertia);

  // foot_link gan cung vao shank_link, vi vay phai gop khoi luong, COM va quan
  // tinh cua hai link. Khong cong truc tiep hai ma tran quan tinh: foot da duoc
  // doi ve COM chung bang dinh ly truc song song.
  Mat3<T> kneeRotationalInertia;
  kneeRotationalInertia <<
      T(0.000356481951), T(-0.000000000000228198435),
      T(0.0000000000272108397), T(-0.000000000000228198435),
      T(0.000359155097), T(0.00000000000128639542),
      T(0.0000000000272108397), T(0.00000000000128639542),
      T(0.00000844284972);
  Vec3<T> kneeCOM(T(-0.00000000452506841), T(-0.000000000212981693),
                  T(-0.110770261859659));
  SpatialInertia<T> kneeInertia(T(0.0607880473076972), kneeCOM,
                                kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  // bodyInertia da gop base_link va imu_link tai vi tri lap IMU trong URDF.
  // Ma tran nay nam tai COM chung cua cum base + IMU.
  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia <<
      T(0.00597809187), T(-0.00000186981010), T(0.00000195722564),
      T(-0.00000186981010), T(0.00557911797), T(0.000152342939),
      T(0.00000195722564), T(0.000152342939), T(0.00691934544);
  Vec3<T> bodyCOM(T(0.00110328769745308), T(0.00124320309861525),
                  T(-0.0019793373857012));
  SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  cheetah._abadInertia = abadInertia;
  cheetah._hipInertia = hipInertia;
  cheetah._kneeInertia = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia = bodyInertia;

  // locations
  cheetah._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  // Vector vi tri chuan theo truc than: +X huong truoc, +Y huong trai, +Z
  // huong len. withLegSigns() doi dau X cho chan sau va Y cho chan phai.
  cheetah._abadLocation = Vec3<T>(0.1513, 0.0500, 0);  // base -> ab/ad
  cheetah._hipLocation = Vec3<T>(0.0560, 0.0133, 0);   // ab/ad -> hip
  cheetah._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  // Hip -> knee gom do lech ngang 72 mm va do ha xuong 210 mm.
  cheetah._kneeLocation = Vec3<T>(0, 0.0720, -0.2100);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H
