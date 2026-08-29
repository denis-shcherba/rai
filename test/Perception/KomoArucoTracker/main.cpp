#include <Perception/KomoArucoTracker.h>

#include <Kin/frame.h>
#include <Kin/cameraview.h>
#include <Kin/viewer.h>
#include <Perception/aruco.h>

/* A/B comparison of the aruco object tracker on a simulated smooth trajectory:

     LEGACY (red ghost):   plain per-frame solve, publishes ret->x unconditionally
     ROBUST (green ghost): corner outlier gating + q-motion-prior, publishes the last accepted pose

   Both trackers are fed the SAME detections, optionally corrupted with pixel noise
   and injected outliers (one cube marker shifted far off, simulating a misdetection).

   Keys (at the pause, every 100 steps):
     o: toggle outlier injection      n: toggle pixel noise
     g: toggle corner gating          p: toggle motion prior
     q: quit                          any other key: continue                  */

struct TrackStats {
  double posErr=0., degErr=0.;            //current frame
  double posSum=0., degSum=0., posMax=0.; //accumulated
  uint jumps=0, frames=0;
  arr lastPos;

  void update(const arr& q_est, const arr& q_true){
    double dp=0.;
    for(uint k=0;k<3;k++) dp += rai::sqr(q_est(k)-q_true(k));
    posErr = sqrt(dp);
    double dot=0., n1=0., n2=0.;
    for(uint k=3;k<7;k++){ dot += q_est(k)*q_true(k); n1 += rai::sqr(q_est(k)); n2 += rai::sqr(q_true(k)); }
    dot = fabs(dot)/(sqrt(n1*n2)+1e-12);
    if(dot>1.) dot=1.;
    degErr = 2.*acos(dot)*180./RAI_PI;
    arr pos(3);
    for(uint k=0;k<3;k++) pos(k) = q_est(k);
    if(lastPos.N && length(pos-lastPos)>.03) jumps++; //estimate jumped >3cm between frames
    lastPos = pos;
    posSum += posErr;  degSum += degErr;
    if(posErr>posMax) posMax = posErr;
    frames++;
  }
};

void testCompareTrackers(){
  //-- ground-truth world, used for rendering the camera images
  rai::Configuration C;
  C.addFile("station.g");
  rai::Frame *obj = C.getFrame("obj");
  uint qi = obj->joint->qIndex;

  //-- display world with ghost estimates (separate config, so the ghosts don't appear in the rendered images)
  rai::Configuration Cd;
  Cd.addFile("station.g");
  rai::Frame *ghostL = Cd.addFrame("est_legacy", "obj_base");
  ghostL->setShape(rai::ST_ssBox, {.17,.17,.17,.005}).setColor({1.,0.,0.,.4});
  rai::Frame *ghostR = Cd.addFrame("est_robust", "obj_base");
  ghostR->setShape(rai::ST_ssBox, {.17,.17,.17,.005}).setColor({0.,1.,0.,.4});

  //-- two trackers, each on its own copy of the scene
  rai::Configuration C1;  C1.addFile("station.g");
  rai::Configuration C2;  C2.addFile("station.g");
  rai::KomoArucoTracker legacy(C1, "obj");
  rai::KomoArucoTracker robust(C2, "obj");
  robust.opt.cornerGating = true;
  robust.opt.motionPrior = true;

  rai::CameraView V(C);
  FrameL cams;
  for(uint c=0;;c++){
    rai::Frame *f = C.getFrame(STRING("camera_"<<c), false);
    if(!f) break;
    cams.append(f);
  }
  CHECK(cams.N, "station.g contains no cameras");

  auto finder = rai::ArucoFinder();
  finder.verbose = 0;

  bool injectOutliers = true;
  double outlierProb = .15;  //per camera per frame
  double pixelNoise = .5;    //px std dev on all corners

  TrackStats sL, sR;
  byteA rgb;
  floatA depth;

  for(uint t=0;;t++){
    //-- ground truth: smooth trajectory (position circle + tumbling rotation), within the obj joint limits
    double time = .05*t;
    rai::Transformation X = 0;
    X.pos.set(.12*cos(.6*time), .12*sin(.4*time), .1+.05*sin(.9*time));
    rai::Quaternion q1;  q1.setRad(.7*sin(.5*time), rai::Vector(1.,0.,0.));
    rai::Quaternion q2;  q2.setRad(.4*time, rai::Vector(0.,0.,1.));
    X.rot = q2*q1;
    arr q_true = {X.pos.x, X.pos.y, X.pos.z, X.rot.w, X.rot.x, X.rot.y, X.rot.z};

    arr q = C.getJointState();
    for(uint k=0;k<7;k++) q(qi+k) = q_true(k);
    C.setJointState(q);

    //-- render all cameras, detect arucos, optionally corrupt, feed the SAME data to both trackers
    legacy.reset();
    robust.reset();
    uint outliersThisFrame=0;
    V.updateConfiguration(C);
    for(uint c=0;c<cams.N;c++){
      V.selectSensor(cams(c));
      V.computeImageAndDepth(rgb, depth);
      finder.find(rgb);
      intA ids = finder.ids;
      arr pts = finder.pts;
      if(!ids.N) continue;

      if(pixelNoise>0.) rndGauss(pts, pixelNoise, true);
      if(injectOutliers && rnd.uni()<outlierProb){
        //corrupt one detected cube marker: shift all its corners far off, simulating a misdetection
        uintA cand;
        for(uint i=0;i<ids.N;i++) if(legacy.CS.obj_aruco_ids.contains(ids(i))) cand.append(i);
        if(cand.N){
          uint m = cand(rnd(cand.N));
          double ang = 2.*RAI_PI*rnd.uni(), mag = 60.+150.*rnd.uni();
          for(uint j=0;j<pts.d1;j++){ pts(m,j,0) += mag*cos(ang);  pts(m,j,1) += mag*sin(ang); }
          outliersThisFrame++;
        }
      }

      legacy.addMultiPointView(ids, pts, c, false); //rendered images are pinhole: no undistortion
      robust.addMultiPointView(ids, pts, c, false);

      if(finder.rgb_annotated.N) Cd.get_viewer()->setQuad(c, finder.rgb_annotated, 0., c*.25, .25);
    }

    legacy.solve(0);
    robust.solve(0);

    arr qL = legacy.publishedPose();
    arr qR = robust.publishedPose();
    sL.update(qL, q_true);
    sR.update(qR, q_true);

    //-- display
    Cd.setJointState(C.getJointState());
    ghostL->setRelativePose(rai::Transformation(qL));
    ghostR->setRelativePose(rai::Transformation(qR));
    str txt;
    txt <<"step " <<t <<"   outliers(o): " <<(injectOutliers?"ON":"off")
        <<"  noise(n): " <<pixelNoise <<"px"
        <<"  gating(g): " <<(robust.opt.cornerGating?"ON":"off")
        <<"  prior(p): " <<(robust.opt.motionPrior?"ON":"off")
        <<(outliersThisFrame?"   << outlier injected":"")
        <<"\nLEGACY (red):   " <<1000.*sL.posErr <<"mm  " <<sL.degErr <<"deg   jumps>3cm: " <<sL.jumps <<"   maxErr: " <<1000.*sL.posMax <<"mm"
        <<"\nROBUST (green): " <<1000.*sR.posErr <<"mm  " <<sR.degErr <<"deg   jumps>3cm: " <<sR.jumps <<"   maxErr: " <<1000.*sR.posMax <<"mm"
        <<"   corners used: " <<robust.n_corners <<" gated: " <<robust.n_gated
        <<(robust.lost?"  LOST":"") <<(robust.accepted?"":"  rejected");
    int key = Cd.view(false, txt);
    if(key=='q') break;

    if(t && !(t%100)){
      str pauseTxt;
      pauseTxt <<txt <<"\n\n== PAUSED ==  keys: o/n/g/p toggle, q quit, other: continue"
               <<"\nmean posErr   legacy: " <<1000.*sL.posSum/sL.frames <<"mm   robust: " <<1000.*sR.posSum/sR.frames <<"mm"
               <<"\nmean rotErr   legacy: " <<sL.degSum/sL.frames <<"deg   robust: " <<sR.degSum/sR.frames <<"deg";
      key = Cd.view(true, pauseTxt);
      if(key=='q') break;
      if(key=='o') injectOutliers = !injectOutliers;
      if(key=='n') pixelNoise = (pixelNoise>0.? 0. : .5);
      if(key=='g') robust.opt.cornerGating = !robust.opt.cornerGating;
      if(key=='p') robust.opt.motionPrior = !robust.opt.motionPrior;
    }

    rai::wait(.02);
  }

  cout <<"\n=== summary over " <<sL.frames <<" frames ==="
       <<"\nLEGACY:  mean " <<1000.*sL.posSum/sL.frames <<"mm / " <<sL.degSum/sL.frames <<"deg   max " <<1000.*sL.posMax <<"mm   jumps>3cm: " <<sL.jumps
       <<"\nROBUST:  mean " <<1000.*sR.posSum/sR.frames <<"mm / " <<sR.degSum/sR.frames <<"deg   max " <<1000.*sR.posMax <<"mm   jumps>3cm: " <<sR.jumps
       <<endl;
}

int main(int argc, char **argv){
  rai::initCmdLine(argc, argv);

  testCompareTrackers();

  return 0;
}
