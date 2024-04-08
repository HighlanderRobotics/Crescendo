// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class VisionHelper {

  /***
   * To be added
   */
  public static final List<TagCountDeviation> TAG_COUNT_DEVIATION_PARAMS =
      List.of(
          // 1 tag
          new TagCountDeviation(
              new UnitDeviationParams(.25, .4, .9), new UnitDeviationParams(.5, .7, 1.5)),

          // 2 tags
          new TagCountDeviation(
              new UnitDeviationParams(.35, .1, .4), new UnitDeviationParams(.5, .7, 1.5)),

          // 3+ tags
          new TagCountDeviation(
              new UnitDeviationParams(.25, .07, .25), new UnitDeviationParams(.15, 1, 1.5)));

  public class Logging {
    public static void logPhotonTrackedTarget(
        PhotonTrackedTarget target, LogTable table, String name) {
      logTransform3d(target.getBestCameraToTarget(), table, name);
      logTransform3d(target.getAlternateCameraToTarget(), table, "Alt " + name);
      logCorners(target, table, name);

      table.put("Tags/Yaw " + name, target.getYaw());
      table.put("Tags/Pitch " + name, target.getPitch());
      table.put("Tags/Area " + name, target.getArea());
      table.put("Tags/Skew " + name, target.getSkew());
      table.put("Tags/Fiducial ID " + name, target.getFiducialId());
      table.put("Tags/Pose Ambiguity " + name, target.getPoseAmbiguity());
    }

    public static void logCorners(PhotonTrackedTarget target, LogTable table, String name) {
      double[] detectedCornersX = new double[4];
      double[] detectedCornersY = new double[4];
      double[] minAreaRectCornersX = new double[4];
      double[] minAreaRectCornersY = new double[4];

      for (int i = 0; i < 4; i++) {
        detectedCornersX[i] = target.getDetectedCorners().get(i).x;
        detectedCornersY[i] = target.getDetectedCorners().get(i).y;
        minAreaRectCornersX[i] = target.getMinAreaRectCorners().get(i).x;
        minAreaRectCornersY[i] = target.getMinAreaRectCorners().get(i).y;
      }
      table.put("Tags/Detected Corners X " + name, detectedCornersX);
      table.put("Tags/Detected Corners Y " + name, detectedCornersY);
      table.put("Tags/Min Area Rect Corners X " + name, minAreaRectCornersX);
      table.put("Tags/Min Area Rect Corners Y " + name, minAreaRectCornersY);
    }

    public static void logTransform3d(Transform3d transform3d, LogTable table, String name) {
      double rotation[] = new double[4];
      rotation[0] = transform3d.getRotation().getQuaternion().getW();
      rotation[1] = transform3d.getRotation().getQuaternion().getX();
      rotation[2] = transform3d.getRotation().getQuaternion().getY();
      rotation[3] = transform3d.getRotation().getQuaternion().getZ();
      table.put("Tags/Rotation " + name, rotation);

      double translation[] = new double[3];
      translation[0] = transform3d.getTranslation().getX();
      translation[1] = transform3d.getTranslation().getY();
      translation[2] = transform3d.getTranslation().getZ();
      table.put("Tags/Translation " + name, translation);
    }

    public static void logVisionConstants(VisionConstants constants, LogTable table) {
      table.put("Vision Constants/Name ", constants.cameraName());
      table.put("Vision Constants/Transform ", constants.robotToCamera());
      table.put("Vision Constants/Intrinsics ", constants.intrinsicsMatrix().getData());
      table.put("Vision Constants/Distortion ", constants.distCoeffs().getData());
    }

    public static Transform3d getLoggedTransform3d(double[] translation, double[] rotation) {
      Transform3d transform3d =
          new Transform3d(
              new Translation3d(translation[0], translation[1], translation[2]),
              new Rotation3d(new Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])));
      return transform3d;
    }

    public static Transform3d getLoggedTransform3d(LogTable table, String name) {
      double[] rotation = table.get("Tags/Rotation " + name, new double[4]);
      double[] translation = table.get("Tags/Translation " + name, new double[3]);
      return getLoggedTransform3d(translation, rotation);
    }

    public static PhotonTrackedTarget getLoggedPhotonTrackedTarget(LogTable table, String name) {
      double[] translation = table.get("Tags/Translation " + name, new double[3]);
      double[] rotation = table.get("Tags/Rotation " + name, new double[4]);
      double[] altTranslation = table.get("Tags/Translation Alt " + name, new double[3]);
      double[] altRotation = table.get("Tags/Rotation Alt " + name, new double[4]);
      double[] detectedCornersX = table.get("Tags/Detected Corners X " + name, new double[4]);
      double[] detectedCornersY = table.get("Tags/Detected Corners Y " + name, new double[4]);
      double[] minAreaRectCornersX =
          table.get("Tags/Min Area Rect Corners X " + name, new double[4]);
      double[] minAreaRectCornersY =
          table.get("Tags/Min Area Rect Corners Y " + name, new double[4]);
      List<TargetCorner> detectedCorners = new ArrayList<>();
      List<TargetCorner> minAreaRectCorners = new ArrayList<>();

      for (int i = 0; i < 4; i++) {
        detectedCorners.add(new TargetCorner(detectedCornersX[i], detectedCornersY[i]));
        minAreaRectCorners.add(new TargetCorner(minAreaRectCornersX[i], minAreaRectCornersY[i]));
      }
      Transform3d pose = getLoggedTransform3d(translation, rotation);
      Transform3d altPose = getLoggedTransform3d(altTranslation, altRotation);
      return (new PhotonTrackedTarget(
          table.get("Tags/Yaw " + name, -1),
          table.get("Tags/Pitch " + name, -1),
          table.get("Tags/Area " + name, -1),
          table.get("Tags/Skew " + name, -1),
          (int) (table.get("Tags/Fiducial ID " + name, -1)),
          pose,
          altPose,
          table.get("Tags/Pose Ambiguity " + name, -1),
          minAreaRectCorners,
          detectedCorners));
    }

    public static VisionConstants getLoggedVisionConstants(LogTable table) {
      return new VisionConstants(
          table.get("Vision Constants Name ", "Default"),
          table.get("Vision Constants Transform ", new Transform3d()),
          new Matrix<N3, N3>(
              Nat.N3(),
              Nat.N3(),
              table.get("Vision Constants Intrinsics ", Matrix.eye(Nat.N3()).getData())),
          new Matrix<N5, N1>(
              Nat.N5(),
              Nat.N1(),
              table.get("Vision Constants Distortion ", new double[] {0.0, 0.0, 0.0, 0.0, 0.0})));
    }
  }

  /**
   * Updates the estimated position of the robot. Returns empty if:
   *
   * <ul>
   *   <li>The timestamp of the provided pipeline result is the same as in the previous call to
   *       {@code update()}.
   *   <li>No targets were found in the pipeline results.
   * </ul>
   *
   * @param cameraResult The latest pipeline result from the camera.
   * @param cameraMatrix Camera calibration data that can be used in the case of no assigned
   *     PhotonCamera.
   * @param distCoeffs Camera calibration data that can be used in the case of no assigned
   *     PhotonCamera
   * @param strat The selected pose solving strategy.
   * @param robotToCamera Transform3d from the center of the robot to the camera mount position (ie,
   *     robot ➔ camera) in the <a href=
   *     "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *     Coordinate System</a>.
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to
   *     create the estimate.
   */
  public static Optional<EstimatedRobotPose> update(
      PhotonPipelineResult cameraResult,
      Matrix<N3, N3> cameraMatrix,
      Matrix<N5, N1> distCoeffs,
      PoseStrategy strat,
      Transform3d robotToCamera,
      Transform3d bestTF) {
    Optional<EstimatedRobotPose> estimatedPose;
    switch (strat) {
      case LOWEST_AMBIGUITY:
        estimatedPose = lowestAmbiguityStrategy(cameraResult, robotToCamera);
        break;
      case MULTI_TAG_PNP_ON_RIO:
        estimatedPose =
            multiTagOnRioStrategy(
                cameraResult,
                Optional.of(cameraMatrix),
                Optional.of(distCoeffs),
                SwerveSubsystem.fieldTags,
                PoseStrategy.LOWEST_AMBIGUITY,
                robotToCamera);
        break;
      case MULTI_TAG_PNP_ON_COPROCESSOR:
        estimatedPose =
            multiTagOnCoprocStrategy(
                cameraResult,
                Optional.of(cameraMatrix),
                Optional.of(distCoeffs),
                robotToCamera,
                PoseStrategy.LOWEST_AMBIGUITY,
                bestTF);
        break;
      default:
        DriverStation.reportError(
            "[PhotonPoseEstimator] Unknown Position Estimation Strategy!", false);
        return Optional.empty();
    }

    return estimatedPose;
  }

  /**
   * Runs SolvePNP on the roborio
   *
   * @param result The latest pipeline result from the camera.
   * @param cameraMatrix Camera calibration data that can be used in the case of no assigned
   *     PhotonCamera.
   * @param distCoeffs Camera calibration data that can be used in the case of no assigned
   *     PhotonCamera
   * @param robotToCamera Transform3d from the center of the robot to the camera mount position (ie,
   *     robot ➔ camera) in the <a href=
   *     "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *     Coordinate System</a>.
   * @param multiTagFallbackStrategy Fallback strategy in case the rio fails. Should usually be
   *     lowest ambiguity
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to
   *     create the estimate.
   */
  private static Optional<EstimatedRobotPose> multiTagOnRioStrategy(
      PhotonPipelineResult result,
      Optional<Matrix<N3, N3>> cameraMatrix,
      Optional<Matrix<N5, N1>> distCoeffs,
      AprilTagFieldLayout fieldTags,
      PoseStrategy multiTagFallbackStrategy,
      Transform3d robotToCamera) {
    boolean hasCalibData = cameraMatrix.isPresent() && distCoeffs.isPresent();
    // cannot run multitagPNP, use fallback strategy
    if (!hasCalibData || result.getTargets().size() < 2) {
      return update(
          result,
          cameraMatrix.get(),
          distCoeffs.get(),
          multiTagFallbackStrategy,
          robotToCamera,
          new Transform3d());
    }

    PNPResult pnpResult =
        VisionEstimation.estimateCamPosePNP(
            cameraMatrix.get(),
            distCoeffs.get(),
            result.getTargets(),
            SwerveSubsystem.fieldTags,
            TargetModel.kAprilTag36h11);
    // try fallback strategy if solvePNP fails for some reason
    if (!pnpResult.isPresent)
      return update(
          result,
          cameraMatrix.get(),
          distCoeffs.get(),
          multiTagFallbackStrategy,
          robotToCamera,
          new Transform3d());
    var best =
        new Pose3d()
            .plus(pnpResult.best) // field-to-camera
            .plus(robotToCamera.inverse()); // field-to-robot

    return Optional.of(
        new EstimatedRobotPose(
            best,
            result.getTimestampSeconds(),
            result.getTargets(),
            PoseStrategy.MULTI_TAG_PNP_ON_RIO));
  }

  /**
   * Runs SolvePNP on a coprocessor
   *
   * @param result The latest pipeline result from the camera.
   * @param cameraMatrix Camera calibration data that can be used in the case of no assigned
   *     PhotonCamera.
   * @param distCoeffs Camera calibration data that can be used in the case of no assigned
   *     PhotonCamera
   * @param robotToCamera Transform3d from the center of the robot to the camera mount position (ie,
   *     robot ➔ camera) in the <a href=
   *     "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *     Coordinate System</a>.
   * @param multiTagFallbackStrategy Fallback strategy in case the coproc fails. Should usually be
   *     the rio or lowest ambiguity
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to
   *     create the estimate.
   */
  private static Optional<EstimatedRobotPose> multiTagOnCoprocStrategy(
      PhotonPipelineResult result,
      Optional<Matrix<N3, N3>> cameraMatrix,
      Optional<Matrix<N5, N1>> distCoeffs,
      Transform3d robotToCamera,
      PoseStrategy multiTagFallbackStrategy,
      Transform3d bestTF) {
    if (!bestTF.equals(new Transform3d())) {
      var best_tf = bestTF;
      var best =
          new Pose3d()
              .plus(best_tf) // field-to-camera
              .relativeTo(SwerveSubsystem.fieldTags.getOrigin())
              .plus(robotToCamera.inverse()); // field-to-robot
      return Optional.of(
          new EstimatedRobotPose(
              best,
              result.getTimestampSeconds(),
              result.getTargets(),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
    } else {
      return update(
          result,
          cameraMatrix.get(),
          distCoeffs.get(),
          multiTagFallbackStrategy,
          robotToCamera,
          new Transform3d());
    }
  }

  /**
   * Return the estimated position of the robot with the lowest position ambiguity from a List of
   * pipeline results.
   *
   * @param result pipeline result
   * @return the estimated position of the robot in the FCS and the estimated timestamp of this
   *     estimation.
   */
  private static Optional<EstimatedRobotPose> lowestAmbiguityStrategy(
      PhotonPipelineResult result, Transform3d robotToCamera) {
    PhotonTrackedTarget lowestAmbiguityTarget = null;

    double lowestAmbiguityScore = 10;

    for (PhotonTrackedTarget target : result.targets) {
      double targetPoseAmbiguity = target.getPoseAmbiguity();

      if (target.getFiducialId() < 1 || target.getFiducialId() > 16) continue;

      if (target.getBestCameraToTarget().getTranslation().getNorm() > 5) continue;

      // Make sure the target is a Fiducial target.
      if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
        lowestAmbiguityScore = targetPoseAmbiguity;
        lowestAmbiguityTarget = target;
      }
    }

    // Although there are confirmed to be targets, none of them may be fiducial
    // targets.
    if (lowestAmbiguityTarget == null) return Optional.empty();
    int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

    Optional<Pose3d> targetPosition = SwerveSubsystem.fieldTags.getTagPose(targetFiducialId);

    if (targetPosition.isEmpty()) {
      DriverStation.reportError(
          "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + targetFiducialId,
          false);
      return Optional.empty();
    }
    var estimatedRobotPose =
        new EstimatedRobotPose(
            targetPosition
                .get()
                .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                .transformBy(robotToCamera.inverse()),
            result.getTimestampSeconds(),
            result.getTargets(),
            PoseStrategy.LOWEST_AMBIGUITY);
    return Optional.of(estimatedRobotPose);
  }

  /*** 5026 - to be tuned if necessary*/
  public static record UnitDeviationParams(
      double distanceMultiplier, double eulerMultiplier, double minimum) {
    private double computeUnitDeviation(double averageDistance) {
      return Math.max(minimum, eulerMultiplier * Math.exp(averageDistance * distanceMultiplier));
    }
  }

  /*** 5026 - to be tuned if necessary*/
  public static record TagCountDeviation(
      UnitDeviationParams xParams, UnitDeviationParams yParams, UnitDeviationParams thetaParams) {
    private Matrix<N3, N1> computeDeviation(double averageDistance) {
      return MatBuilder.fill(
          Nat.N3(),
          Nat.N1(),
          xParams.computeUnitDeviation(averageDistance),
          yParams.computeUnitDeviation(averageDistance),
          thetaParams.computeUnitDeviation(averageDistance));
    }

    public TagCountDeviation(UnitDeviationParams xyParams, UnitDeviationParams thetaParams) {
      this(xyParams, xyParams, thetaParams);
    }
  }

  public static record VisionMeasurement(
      EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {}

  /*** 5026 - to be tuned if necessary*/
  public static Matrix<N3, N1> findVisionMeasurementStdDevs(EstimatedRobotPose estimation) {
    double sumDistance = 0;
    for (var target : estimation.targetsUsed) {
      var t3d = target.getBestCameraToTarget();
      sumDistance +=
          Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
    }
    double avgDistance = sumDistance / estimation.targetsUsed.size();

    var deviation = Vision.visionPointBlankDevs.times(avgDistance * Vision.distanceFactor);
    if (estimation.targetsUsed.size() == 1) {
      deviation = deviation.times(2);
    }
    // TAG_COUNT_DEVIATION_PARAMS
    //     .get(
    //         MathUtil.clamp(
    //             estimation.targetsUsed.size() - 1, 0, TAG_COUNT_DEVIATION_PARAMS.size() - 1))
    //     .computeDeviation(avgDistance);

    return deviation;
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public static Field2d getSimDebugField(VisionSystemSim visionSim) {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
