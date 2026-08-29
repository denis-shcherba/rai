#pragma once

#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Control/CtrlMsgs.h>

#include <vector>

namespace rai {

struct ArucoThread;

//===========================================================================

struct CalibrationScene {
  Configuration& C;

  FrameL cams;
  FrameL arucos;
  FrameL calibs;
  FrameL calibs_joints;

  arrA Fxycxy;
  arrA Distortion;
  Frame * obj;
  uintA obj_aruco_ids;

  CalibrationScene(Configuration& C, const char* obj_name=0);

  //-- setup calib dof frames
  void addCalibDofs_arucos();
  void addCalibDofs_cameras();
  void addCalibDofs_joints(const uintA& jointIds);

  str report();
};

//===========================================================================

void komoCalibrate(CalibrationScene& CS,
		   const intAA& ids, const arrA& pts, const arr& qs,
		   bool calibrate_cams = true,
		   bool calibrate_arucos = true,
		   bool calibrate_joints = true,
		   bool calibrate_objPoses = false,
		   bool undistort_points = true,
		   double calib_joint_regularization = 1e1);

//===========================================================================

struct NaiveTrackerFilter {
    arr q, qdel;
    double good_ratio=0;
    double threshold;
    double alpha=.7, beta=.3, gamma=.1;
    double err_filtered;

    NaiveTrackerFilter(double threshold=.02);

    void update(const arr& q_measured);
};

//===========================================================================

struct KomoArucoTrackerOptions {
  //-- corner outlier gating: reject corner observations inconsistent with the warm-start (=last solution) prediction
  RAI_PARAM("arucoTracker/", bool, cornerGating, false)
  RAI_PARAM("arucoTracker/", double, gate_angle, .02)     //rad; angular gate ~ pixel_error/focal_length (.02 ~ 20px at f=1000)
  RAI_PARAM("arucoTracker/", double, gate_lostRatio, .5)  //if more corners than this ratio fail the gate, assume tracking lost: accept all corners, drop the prior
  //-- motion prior: weak sos objective pulling the obj dofs towards the last accepted estimate
  RAI_PARAM("arucoTracker/", bool, motionPrior, false)
  RAI_PARAM("arucoTracker/", double, priorWeight, 5.)     //compare: point view objectives have weight 1e2
  //-- acceptance test in robust mode (replaces legacy ret->sos<10, which ignores the number of corners)
  RAI_PARAM("arucoTracker/", double, sosPerCorner, .25)
};

//===========================================================================

struct KomoArucoTracker{
  CalibrationScene CS;

  std::shared_ptr<KOMO> komo;
  std::shared_ptr<SolverReturn> ret;
  NaiveTrackerFilter filter;

  KomoArucoTrackerOptions opt;

  //-- state & diagnostics of the robust mode (valid after solve())
  arr q_accepted;              //last accepted estimate; the published pose in robust mode
  bool accepted=false;         //last solve passed the acceptance test
  bool lost=false;             //gate declared tracking lost (accepted all corners, dropped prior)
  uint n_corners=0, n_gated=0; //corners used as objectives / rejected by the gate

  KomoArucoTracker(Configuration& C, const char* obj_name) : CS(C, obj_name) {}

  void reset(bool force_contructor=false);
  void addArucoDetected(uint cam_id, uint aruco_id);
  void addPointView(arr p, uint cam_id, uint aruco_id, uint corner_id);
  void addMultiPointView(const intA& ids, const arr& pts, uint cam_id, bool undistort_points = true);

  void solve(int verbose=0, double tolerance=1e-4);

  arr publishedPose(); //robust mode: last accepted pose; legacy mode: ret->x

private:
  //angular error (sin of angle) between the pixel's viewing ray and the corner predicted from the warm start; <0: no prediction available
  double cornerGateError(const arr& p, uint cam_id, uint aruco_id, uint corner_id);

  struct PendingCorner { arr p; uint cam_id, aruco_id, corner_id; double err; };
  std::vector<PendingCorner> pending; //observations buffered for gating, flushed in solve()
};

//===========================================================================

struct KomoArucoTracker_Thread : Thread {
    Array<std::shared_ptr<ArucoThread>> aruco_threads;
    Var<CtrlStateMsg>& state;
    Var<arr> obj_pose;
    KomoArucoTracker tracker;

    KomoArucoTracker_Thread(const Array<std::shared_ptr<ArucoThread>>& aruco_threads,
                            Var<CtrlStateMsg>& state,
                            Configuration& C, const char* obj_name);
    ~KomoArucoTracker_Thread();

    void step();
};

} //namespace
