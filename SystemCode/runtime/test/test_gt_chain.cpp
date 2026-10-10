// End-to-end check of the ground-truth chain, with no rig and no printed trays:
//
//   T_world_object = T_world_cam . T_cam_marker . T_marker_object
//
// A marker is rendered into a synthetic image at a pose derived from a CHOSEN
// object pose, and the chain must recover that object pose. This is the test
// that would catch a wrong frame convention, a transposed rotation, or a
// marker-to-object offset applied with the wrong sign -- every one of which
// produces ground truth that looks completely healthy and silently biases every
// ADD-S number measured against it.
#include <gtest/gtest.h>

#ifdef THE_EYES_WITH_ARUCO

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "the_eyes/calibration/marker_detector.hpp"
#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/runtime/marker_trays.hpp"

using namespace the_eyes;
using namespace the_eyes::runtime;

namespace {

Eigen::Matrix3d TestK() {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = 1365.78;  // the rig's real 1920x1080 factory intrinsics
  K(1, 1) = 1365.27;
  K(0, 2) = 970.32;
  K(1, 2) = 546.02;
  return K;
}

// Renders one marker of `dict` at T_cam_marker into a blank 1920x1080 frame.
cv::Mat RenderMarker(int id, double side_m, const Eigen::Isometry3d& T_cam_marker,
                     const Eigen::Matrix3d& K) {
  cv::Mat tag;
  auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
  cv::aruco::drawMarker(dict, id, 400, tag, 1);
  cv::cvtColor(tag, tag, cv::COLOR_GRAY2BGR);

  // Marker frame: +x right, +y up, +z out of the face, origin at the centre.
  // Image order is TL, TR, BR, BL, matching estimatePoseSingleMarkers.
  const double h = side_m / 2.0;
  const std::vector<Eigen::Vector3d> obj = {
      {-h, h, 0}, {h, h, 0}, {h, -h, 0}, {-h, -h, 0}};
  std::vector<cv::Point2f> dst;
  for (const Eigen::Vector3d& p : obj) {
    const Eigen::Vector3d c = T_cam_marker * p;
    EXPECT_GT(c.z(), 0.0) << "marker must be in front of the camera";
    const Eigen::Vector3d uv = K * (c / c.z());
    dst.emplace_back(static_cast<float>(uv.x()), static_cast<float>(uv.y()));
  }
  const std::vector<cv::Point2f> src = {{0, 0},
                                        {static_cast<float>(tag.cols - 1), 0},
                                        {static_cast<float>(tag.cols - 1),
                                         static_cast<float>(tag.rows - 1)},
                                        {0, static_cast<float>(tag.rows - 1)}};
  cv::Mat frame(1080, 1920, CV_8UC3, cv::Scalar(190, 190, 190));
  cv::Mat H = cv::getPerspectiveTransform(src, dst);
  cv::warpPerspective(tag, frame, H, frame.size(), cv::INTER_CUBIC, cv::BORDER_TRANSPARENT);
  return frame;
}

// Draws one more marker onto an existing frame, so a whole tray can be rendered.
void RenderMarkerInto(cv::Mat& frame, int id, double side_m,
                      const Eigen::Isometry3d& T_cam_marker, const Eigen::Matrix3d& K) {
  cv::Mat tag;
  auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
  cv::aruco::drawMarker(dict, id, 400, tag, 1);
  cv::cvtColor(tag, tag, cv::COLOR_GRAY2BGR);
  const double h = side_m / 2.0;
  const std::vector<Eigen::Vector3d> obj = {
      {-h, h, 0}, {h, h, 0}, {h, -h, 0}, {-h, -h, 0}};
  std::vector<cv::Point2f> dst;
  for (const Eigen::Vector3d& p : obj) {
    const Eigen::Vector3d c = T_cam_marker * p;
    EXPECT_GT(c.z(), 0.0);
    const Eigen::Vector3d uv = K * (c / c.z());
    dst.emplace_back(static_cast<float>(uv.x()), static_cast<float>(uv.y()));
  }
  const std::vector<cv::Point2f> src = {{0, 0},
                                        {static_cast<float>(tag.cols - 1), 0},
                                        {static_cast<float>(tag.cols - 1),
                                         static_cast<float>(tag.rows - 1)},
                                        {0, static_cast<float>(tag.rows - 1)}};
  cv::Mat H = cv::getPerspectiveTransform(src, dst);
  cv::warpPerspective(tag, frame, H, frame.size(), cv::INTER_CUBIC, cv::BORDER_TRANSPARENT);
}

Eigen::Isometry3d Translation(const Eigen::Vector3d& t) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = t;
  return T;
}

}  // namespace

TEST(GroundTruthChain, RecoversAChosenObjectPoseThroughTheWholeChain) {
  const TrayConfig trays = LoadTrayConfig(std::string(THE_EYES_CONFIG_DIR) + "/marker_trays.yaml");
  const fusion::ExtrinsicsSet ex =
      fusion::LoadExtrinsics(std::string(THE_EYES_CONFIG_DIR) + "/extrinsics.yaml");
  const Eigen::Isometry3d T_world_cam = ex.cameras.at("cam_front").T_world_cam;
  const Eigen::Matrix3d K = TestK();

  // A red_cube seated on the table, rotated so a wrong rotation convention
  // cannot pass by symmetry.
  Eigen::Isometry3d T_world_object = Eigen::Isometry3d::Identity();
  T_world_object.linear() =
      Eigen::AngleAxisd(37.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_world_object.translation() = Eigen::Vector3d(0.02, -0.01, 0.025);

  const Tray& tray = trays.trays.at("red_cube");
  calibration::MarkerDetector det(trays.dictionary, trays.marker_length_m);

  for (const TrayMarker& mk : tray.markers) {
    const Eigen::Isometry3d T_marker_object = MarkerToObject(tray, mk.id);
    const Eigen::Isometry3d T_world_marker = T_world_object * T_marker_object.inverse();
    const Eigen::Isometry3d T_cam_marker = T_world_cam.inverse() * T_world_marker;

    const cv::Mat frame = RenderMarker(mk.id, trays.marker_length_m, T_cam_marker, K);
    const auto obs = det.Detect(frame, K);
    ASSERT_EQ(obs.size(), 1u) << "marker " << mk.id << " was not detected";
    EXPECT_EQ(obs[0].id, mk.id);
    EXPECT_LT(obs[0].reprojection_px, 1.5);

    const Eigen::Isometry3d recovered =
        T_world_cam * obs[0].T_cam_marker * MarkerToObject(tray, mk.id);
    const double dt = (recovered.translation() - T_world_object.translation()).norm();
    const double dr =
        Eigen::AngleAxisd(recovered.linear().transpose() * T_world_object.linear()).angle() *
        180.0 / M_PI;
    // These bounds are deliberately loose. A single 40 mm marker at the rig's
    // 0.7 m and ~55 degrees of incidence carries 1-3 mm of genuine pose noise
    // (measured on synthetic renders at several resolutions, so it is the
    // method's floor and not this renderer's). What the test is for is the
    // failure that is NOT noise: a transposed rotation or a marker-to-object
    // offset applied with the wrong sign puts the object 114 mm away -- the
    // spacing of the two cube markers -- or rotates it by 90 degrees.
    //
    // 10 mm, not the 6 mm this started at: the camera pose is the SHIPPED
    // extrinsics, so the noise moves with every recalibration. Measured 4.2 /
    // 3.3 mm on the 2026-09-20 geometry and 1.3 / 2.3 mm on 2026-10-04, and the
    // two-marker average below went 4.8 -> 6.07 mm the other way and tripped a
    // 6 mm bound on a rig that was fine. 10 mm still sits 11x under the 114 mm
    // this is here to catch.
    EXPECT_LT(dt, 0.010) << "marker " << mk.id << ": translation off by " << dt * 1000 << " mm";
    EXPECT_LT(dr, 3.0) << "marker " << mk.id << ": rotation off by " << dr << " deg";
  }
}

TEST(GroundTruthChain, BothMarkersOnATrayAgreeOnTheObject) {
  // The two red_cube markers sit 114 mm apart and their t_tray_marker values
  // differ in sign. If the offset were applied the wrong way round, each marker
  // would place the object on the wrong side and they would disagree by 114 mm
  // -- while each on its own still looked plausible.
  const TrayConfig trays = LoadTrayConfig(std::string(THE_EYES_CONFIG_DIR) + "/marker_trays.yaml");
  const fusion::ExtrinsicsSet ex =
      fusion::LoadExtrinsics(std::string(THE_EYES_CONFIG_DIR) + "/extrinsics.yaml");
  const Eigen::Isometry3d T_world_cam = ex.cameras.at("cam_front").T_world_cam;
  const Eigen::Matrix3d K = TestK();
  const Tray& tray = trays.trays.at("red_cube");

  Eigen::Isometry3d T_world_object = Eigen::Isometry3d::Identity();
  T_world_object.translation() = Eigen::Vector3d(0.0, 0.0, 0.025);

  calibration::MarkerDetector det(trays.dictionary, trays.marker_length_m);
  std::vector<Eigen::Isometry3d> recovered;
  for (const TrayMarker& mk : tray.markers) {
    const Eigen::Isometry3d T_world_marker = T_world_object * MarkerToObject(tray, mk.id).inverse();
    const cv::Mat frame =
        RenderMarker(mk.id, trays.marker_length_m, T_world_cam.inverse() * T_world_marker, K);
    const auto obs = det.Detect(frame, K);
    ASSERT_EQ(obs.size(), 1u);
    recovered.push_back(T_world_cam * obs[0].T_cam_marker * MarkerToObject(tray, mk.id));
  }
  ASSERT_EQ(recovered.size(), 2u);
  const double disagreement = (recovered[0].translation() - recovered[1].translation()).norm();
  EXPECT_LT(disagreement, 0.008) << "the two markers place the object " << disagreement * 1000
                                 << " mm apart; a sign error on t_tray_marker would make this 114";

  const PoseAverage avg = AveragePoses(recovered);
  EXPECT_EQ(avg.n, 2);
  // Both markers are seen by ONE camera, so they share its depth bias and the
  // average does not cancel it -- 4.8 mm on the 2026-09-20 geometry, 6.07 mm on
  // 2026-10-04. Same 10 mm bound, for the same reason, as the test above.
  EXPECT_LT((avg.mean.translation() - T_world_object.translation()).norm(), 0.010);
}

TEST(GroundTruthChain, TablePlaneSolveBeatsTheFreeSolveOnTheSameCorners) {
  // The cube tray flat on the table, both of its markers rendered into each of
  // the rig's two real views. The SAME detected corners then go through both
  // solves. The free one has to guess depth and tilt from a 40 mm square; the
  // table-plane one is told the height and solves only x, y and yaw.
  const TrayConfig trays = LoadTrayConfig(std::string(THE_EYES_CONFIG_DIR) + "/marker_trays.yaml");
  const fusion::ExtrinsicsSet ex =
      fusion::LoadExtrinsics(std::string(THE_EYES_CONFIG_DIR) + "/extrinsics.yaml");
  const Eigen::Matrix3d K = TestK();
  const Tray& tray = trays.trays.at("red_cube");
  const auto marker_xy = MarkerCornersXY(trays.marker_length_m);

  Eigen::Isometry3d T_world_tray = Eigen::Isometry3d::Identity();
  T_world_tray.linear() =
      Eigen::AngleAxisd(25.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_world_tray.translation() = Eigen::Vector3d(-0.05, 0.03, tray.plate_thickness_m);
  const Eigen::Isometry3d T_world_object = T_world_tray * Translation(tray.t_tray_object);

  calibration::MarkerDetector det(trays.dictionary, trays.marker_length_m);
  double free_sum = 0.0, plane_sum = 0.0, plane_max = 0.0, plane_rms_max = 0.0;
  int n = 0;
  for (const char* cam_id : {"cam_front", "cam_over"}) {
    const Eigen::Isometry3d T_world_cam = ex.cameras.at(cam_id).T_world_cam;
    cv::Mat frame;
    for (const TrayMarker& mk : tray.markers) {
      const Eigen::Isometry3d T_cam_marker =
          T_world_cam.inverse() * T_world_tray * Translation(mk.t_tray_marker);
      if (frame.empty()) frame = RenderMarker(mk.id, trays.marker_length_m, T_cam_marker, K);
      else RenderMarkerInto(frame, mk.id, trays.marker_length_m, T_cam_marker, K);
    }
    const auto obs = det.Detect(frame, K);
    ASSERT_EQ(obs.size(), tray.markers.size()) << cam_id;
    for (const auto& m : obs) {
      const Eigen::Isometry3d free_pose = T_world_cam * m.T_cam_marker * MarkerToObject(tray, m.id);
      double marker_z = 0.0;
      for (const TrayMarker& mk : tray.markers) {
        if (mk.id == m.id) marker_z = tray.plate_thickness_m + mk.t_tray_marker.z();
      }
      std::vector<Eigen::Vector2d> model(marker_xy.begin(), marker_xy.end()), world;
      for (const auto& c : m.corners) {
        world.push_back(PixelOnWorldPlane(c, K, T_world_cam, marker_z).head<2>());
      }
      const PlanarFit fit = FitPlanarRigid(model, world);
      const Eigen::Isometry3d plane_pose = PoseOnPlane(fit, marker_z) * MarkerToObject(tray, m.id);

      const double e_free = (free_pose.translation() - T_world_object.translation()).norm();
      const double e_plane = (plane_pose.translation() - T_world_object.translation()).norm();
      free_sum += e_free;
      plane_sum += e_plane;
      plane_max = std::max(plane_max, e_plane);
      plane_rms_max = std::max(plane_rms_max, fit.rms_m);
      ++n;
      // Height is not estimated at all, so it is exact.
      EXPECT_NEAR(plane_pose.translation().z(), T_world_object.translation().z(), 1e-12);
    }
  }
  ASSERT_EQ(n, 4);
  RecordProperty("free_mean_mm", std::to_string(free_sum / n * 1000));
  RecordProperty("plane_mean_mm", std::to_string(plane_sum / n * 1000));
  // Loose on purpose: the camera poses are the SHIPPED extrinsics, so these
  // move with every recalibration (see the 10 mm note above).
  EXPECT_LT(plane_max, 0.0015) << "worst table-plane error " << plane_max * 1000 << " mm";
  EXPECT_LT(plane_sum, 0.5 * free_sum)
      << "table-plane mean " << plane_sum / n * 1000 << " mm vs free mean "
      << free_sum / n * 1000 << " mm";
  // 1 mm, not the 0.5 mm this started at, for the same reason: it passed on
  // the calibrations up to 2026-10-10 16:33 and read 0.54 mm on the two after
  // it (17:16, 20:46), on a rig that was fine. A marker that is not flat on
  // the table misses by millimetres, so 1 mm still tells the two apart.
  EXPECT_LT(plane_rms_max, 0.001) << "a flat marker must fit the plane";
}

TEST(GroundTruthChain, ATrayThatIsNotFlatShowsUpAsPlaneMisfit) {
  // The assumption can be wrong -- a tray resting on a cable -- and the solve
  // has to say so rather than return a confident pose. A marker tipped 6 deg
  // no longer projects onto the plane as a square, and the rigid fit's residual
  // is what reports it.
  const TrayConfig trays = LoadTrayConfig(std::string(THE_EYES_CONFIG_DIR) + "/marker_trays.yaml");
  const fusion::ExtrinsicsSet ex =
      fusion::LoadExtrinsics(std::string(THE_EYES_CONFIG_DIR) + "/extrinsics.yaml");
  const Eigen::Matrix3d K = TestK();
  const Eigen::Isometry3d T_world_cam = ex.cameras.at("cam_front").T_world_cam;
  const auto marker_xy = MarkerCornersXY(trays.marker_length_m);
  calibration::MarkerDetector det(trays.dictionary, trays.marker_length_m);

  auto misfit = [&](double tilt_deg) {
    Eigen::Isometry3d T_world_marker = Eigen::Isometry3d::Identity();
    T_world_marker.linear() =
        Eigen::AngleAxisd(tilt_deg * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
    T_world_marker.translation() = Eigen::Vector3d(0.0, 0.0, 0.0101);
    const cv::Mat frame =
        RenderMarker(100, trays.marker_length_m, T_world_cam.inverse() * T_world_marker, K);
    const auto obs = det.Detect(frame, K);
    EXPECT_EQ(obs.size(), 1u);
    std::vector<Eigen::Vector2d> model(marker_xy.begin(), marker_xy.end()), world;
    for (const auto& c : obs.at(0).corners) {
      world.push_back(PixelOnWorldPlane(c, K, T_world_cam, 0.0101).head<2>());
    }
    return FitPlanarRigid(model, world).rms_m;
  };
  const double flat = misfit(0.0), tipped = misfit(6.0);
  EXPECT_LT(flat, 0.0005);
  EXPECT_GT(tipped, 3.0 * flat) << "flat " << flat * 1000 << " mm, tipped " << tipped * 1000 << " mm";
  EXPECT_GT(tipped, 0.0008);
}

TEST(GroundTruthChain, AveragingAcrossViewsBeatsAnySingleObservation) {
  // The tool does not trust one observation; it pools markers, cameras and
  // frames. This asserts that pooling actually helps -- if it did not, the
  // protocol's 30-frame window would be ceremony.
  const TrayConfig trays = LoadTrayConfig(std::string(THE_EYES_CONFIG_DIR) + "/marker_trays.yaml");
  const fusion::ExtrinsicsSet ex =
      fusion::LoadExtrinsics(std::string(THE_EYES_CONFIG_DIR) + "/extrinsics.yaml");
  const Eigen::Matrix3d K = TestK();
  const Tray& tray = trays.trays.at("red_cube");
  calibration::MarkerDetector det(trays.dictionary, trays.marker_length_m);

  Eigen::Isometry3d T_world_object = Eigen::Isometry3d::Identity();
  T_world_object.translation() = Eigen::Vector3d(0.01, 0.02, 0.025);

  std::vector<Eigen::Isometry3d> all;
  double worst_single = 0.0;
  for (const auto& [cam_id, cam] : ex.cameras) {
    for (const TrayMarker& mk : tray.markers) {
      const Eigen::Isometry3d T_world_marker =
          T_world_object * MarkerToObject(tray, mk.id).inverse();
      const cv::Mat frame = RenderMarker(mk.id, trays.marker_length_m,
                                         cam.T_world_cam.inverse() * T_world_marker, K);
      const auto obs = det.Detect(frame, K);
      if (obs.empty()) continue;
      // Every synthetic view here must be well away from the planar ambiguity;
      // if one is not, the fixture is wrong rather than the code.
      EXPECT_GT(obs[0].ambiguity, 1.05) << cam_id << " marker " << mk.id;
      const Eigen::Isometry3d w =
          cam.T_world_cam * obs[0].T_cam_marker * MarkerToObject(tray, mk.id);
      worst_single = std::max(worst_single,
                              (w.translation() - T_world_object.translation()).norm());
      all.push_back(w);
    }
  }
  ASSERT_GE(all.size(), 4u) << "expected both markers in both cameras";
  const PoseAverage avg = AveragePoses(all);
  const double pooled = (avg.mean.translation() - T_world_object.translation()).norm();
  EXPECT_LE(pooled, worst_single) << "pooling made it worse: " << pooled * 1000 << " mm vs "
                                  << worst_single * 1000 << " mm";
  EXPECT_GT(avg.translation_rms_m, 0.0) << "the spread must be reported, not zero";
}


// Why SolveMultiMarkerPose exists, measured on the estimator itself.
//
// red_cube is the worst case on this rig: two markers only 114 mm apart. A
// single 40 mm marker at this distance has poorly conditioned DEPTH, and
// averaging two independent poses cannot fix conditioning -- it averages the
// noise and leaves the geometry alone. Solving both markers' eight corners in
// one PnP widens the effective target from 40 mm to 114 mm, which is what
// actually constrains depth.
//
// Corners are projected analytically and perturbed, rather than rendered: a
// rendered tag carries ~3 mm of its own rasterisation noise (see the bounds
// above), which is the same floor for both methods and would hide the effect
// being measured. Fixed seed, so this is deterministic.
TEST(GroundTruthChain, JointMultiMarkerSolveIsBetterConditionedThanAveragingPoses) {
  const TrayConfig trays = LoadTrayConfig(std::string(THE_EYES_CONFIG_DIR) + "/marker_trays.yaml");
  const Tray& tray = trays.trays.at("red_cube");
  const Eigen::Matrix3d K = TestK();
  ASSERT_GE(tray.markers.size(), 2u);

  const fusion::ExtrinsicsSet ex =
      fusion::LoadExtrinsics(std::string(THE_EYES_CONFIG_DIR) + "/extrinsics.yaml");
  Eigen::Isometry3d T_world_tray = Eigen::Isometry3d::Identity();
  T_world_tray.linear() =
      Eigen::AngleAxisd(29.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_world_tray.translation() = Eigen::Vector3d(0.02, -0.01, 0.0105);
  const Eigen::Isometry3d T_cam_tray =
      ex.cameras.at("cam_front").T_world_cam.inverse() * T_world_tray;

  const double h = trays.marker_length_m / 2.0;
  const Eigen::Vector3d offset[4] = {{-h, h, 0.0}, {h, h, 0.0}, {h, -h, 0.0}, {-h, -h, 0.0}};
  std::map<int, Eigen::Vector3d> layout;
  for (const TrayMarker& tm : tray.markers) layout[tm.id] = tm.t_tray_marker;

  std::mt19937 rng(20260920u);
  std::normal_distribution<double> jitter(0.0, 0.3);  // px, the order a real corner wobbles
  double sq_joint = 0.0, sq_avg = 0.0;
  const int kTrials = 200;
  for (int trial = 0; trial < kTrials; ++trial) {
    std::vector<calibration::MarkerObservation> obs;
    for (const TrayMarker& tm : tray.markers) {
      calibration::MarkerObservation m;
      m.id = tm.id;
      for (int c = 0; c < 4; ++c) {
        const Eigen::Vector3d p = T_cam_tray * (tm.t_tray_marker + offset[c]);
        const Eigen::Vector3d uv = K * (p / p.z());
        m.corners[c] = Eigen::Vector2d(uv.x() + jitter(rng), uv.y() + jitter(rng));
      }
      obs.push_back(m);
    }

    // The path this replaces: each marker solved alone, then averaged. Solving
    // one marker through the same entry point keeps the comparison honest --
    // only the number of markers in the fit differs.
    std::vector<Eigen::Isometry3d> per_marker;
    for (const calibration::MarkerObservation& m : obs) {
      const std::map<int, Eigen::Vector3d> alone{{m.id, Eigen::Vector3d::Zero()}};
      const calibration::MultiMarkerPose single =
          calibration::SolveMultiMarkerPose({m}, alone, trays.marker_length_m, K);
      per_marker.push_back(single.T_cam_frame * Translation(-layout.at(m.id)));
    }
    const PoseAverage avg = AveragePoses(per_marker);
    const calibration::MultiMarkerPose joint =
        calibration::SolveMultiMarkerPose(obs, layout, trays.marker_length_m, K);
    ASSERT_EQ(joint.markers_used, static_cast<int>(tray.markers.size()));

    sq_avg += (avg.mean.translation() - T_cam_tray.translation()).squaredNorm();
    sq_joint += (joint.T_cam_frame.translation() - T_cam_tray.translation()).squaredNorm();
  }
  const double rms_avg = std::sqrt(sq_avg / kTrials);
  const double rms_joint = std::sqrt(sq_joint / kTrials);

  EXPECT_LT(rms_joint, rms_avg) << "joint " << rms_joint * 1000 << " mm rms vs averaged "
                                << rms_avg * 1000 << " mm rms";
  // MEASURED, and smaller than the geometry argument suggests: 1.63 mm rms
  // against 1.73 mm, about 5%, not the ~3x the 114/40 mm width ratio would
  // imply. Averaging two independent poses already recovers much of what the
  // joint fit does, because their depth errors are independent and partly
  // cancel. The bound is set just inside the measured margin rather than at a
  // hoped-for one -- if a change makes the joint fit merely equal to averaging,
  // that is worth knowing, and this catches it.
  EXPECT_LT(rms_joint, 0.98 * rms_avg)
      << "joint " << rms_joint * 1000 << " mm rms vs averaged " << rms_avg * 1000 << " mm rms";
}

// One visible marker must still work, and must agree with the single-marker
// solve it falls back to -- a tray half out of frame is normal, not an error.
TEST(GroundTruthChain, JointSolveWithOneMarkerMatchesTheSingleMarkerPose) {
  const TrayConfig trays = LoadTrayConfig(std::string(THE_EYES_CONFIG_DIR) + "/marker_trays.yaml");
  const Tray& tray = trays.trays.at("red_cube");
  const Eigen::Matrix3d K = TestK();
  const int id = tray.markers.front().id;

  const fusion::ExtrinsicsSet ex =
      fusion::LoadExtrinsics(std::string(THE_EYES_CONFIG_DIR) + "/extrinsics.yaml");
  Eigen::Isometry3d T_world_marker = Eigen::Isometry3d::Identity();
  T_world_marker.translation() = Eigen::Vector3d(0.01, 0.0, 0.0105);
  const Eigen::Isometry3d T_cam_marker =
      ex.cameras.at("cam_front").T_world_cam.inverse() * T_world_marker;

  const cv::Mat frame = RenderMarker(id, trays.marker_length_m, T_cam_marker, K);
  calibration::MarkerDetector det(trays.dictionary, trays.marker_length_m);
  const std::vector<calibration::MarkerObservation> obs = det.Detect(frame, K);
  ASSERT_EQ(obs.size(), 1u);

  // Its position is the frame origin here, so the joint pose IS the marker pose.
  const std::map<int, Eigen::Vector3d> layout{{id, Eigen::Vector3d::Zero()}};
  const calibration::MultiMarkerPose joint =
      calibration::SolveMultiMarkerPose(obs, layout, trays.marker_length_m, K);

  EXPECT_EQ(joint.markers_used, 1);
  EXPECT_LT((joint.T_cam_frame.translation() - obs[0].T_cam_marker.translation()).norm(), 1e-6);
}

#endif  // THE_EYES_WITH_ARUCO
