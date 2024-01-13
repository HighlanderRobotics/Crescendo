// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class VisionHelper {
  // constants
  public static final Matrix<N3, N3> CAMERA_MATRIX_OPT =
      MatBuilder.fill(
          Nat.N3(),
          Nat.N3(),
          923.5403619629557,
          0.0,
          644.4965658066068,
          0.0,
          925.8136962361125,
          402.6412935350414,
          0.0,
          0.0,
          1.0); // TODO tune maybe
  public static final Matrix<N5, N1> DIST_COEFFS_OPT =
      MatBuilder.fill(
          Nat.N5(),
          Nat.N1(),
          0.05452153950284706,
          -0.04331612051891956,
          0.00176988756858703,
          -0.004530368741385627,
          -0.040501622476628085); // TODO tune maybe
  public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(); // TODO find
  public static final List<TagCountDeviation> TAG_COUNT_DEVIATION_PARAMS =
      List.of(
          // 1 tag
          new TagCountDeviation(
              new UnitDeviationParams(.25, .4, .9),
              new UnitDeviationParams(.35, .5, 1.2),
              new UnitDeviationParams(.5, .7, 1.5)),

          // 2 tags
          new TagCountDeviation(
              new UnitDeviationParams(.35, .1, .4), new UnitDeviationParams(.5, .7, 1.5)),

          // 3+ tags
          new TagCountDeviation(
              new UnitDeviationParams(.25, .07, .25), new UnitDeviationParams(.15, 1, 1.5)));

  /**
   * Poll data from the configured cameras and update the estimated position of the robot. Returns
   * empty if there are no cameras set or no targets were found from the cameras.
   *
   * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
   *     the estimate
   */
  public static Optional<EstimatedRobotPose> update(
      PhotonPipelineResult result,
      AprilTagFieldLayout fieldTags,
      PoseStrategy primaryStrategy,
      PoseStrategy multiTagFallbackStrategy) {

    double poseCacheTimestampSeconds = -1;

    // Time in the past -- give up, since the following if expects times > 0
    if (result.getTimestampSeconds() < 0) {
      return Optional.empty();
    }

    // If the pose cache timestamp was set, and the result is from the same timestamp, return an
    // empty result
    // if (poseCacheTimestampSeconds > 0
    //     && Math.abs(poseCacheTimestampSeconds - result.getTimestampSeconds()) < 1e-6) {
    //   return Optional.empty();
    // }

    // Remember the timestamp of the current result used
    poseCacheTimestampSeconds = result.getTimestampSeconds();

    // If no targets seen, trivial case -- return empty result
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> estimatedPose;
    switch (primaryStrategy) {
      case LOWEST_AMBIGUITY:
        estimatedPose = lowestAmbiguityStrategy(result, fieldTags);
        break;
      case MULTI_TAG_PNP_ON_COPROCESSOR:
        estimatedPose =
            multiTagPNPStrategy(result, fieldTags, primaryStrategy, multiTagFallbackStrategy);
        break;
      default:
        DriverStation.reportError(
            "[PhotonPoseEstimator] Unknown Position Estimation Strategy!", false);
        return Optional.empty();
    }

    return estimatedPose;
  }

  private static Optional<EstimatedRobotPose> multiTagPNPStrategy(
      PhotonPipelineResult result,
      AprilTagFieldLayout fieldTags,
      PoseStrategy primaryStrategy,
      PoseStrategy multiTagFallbackStrategy) {
    // Arrays we need declared up front
    var visCorners = new ArrayList<TargetCorner>();
    var knownVisTags = new ArrayList<AprilTag>();
    var fieldToCams = new ArrayList<Pose3d>();
    var fieldToCamsAlt = new ArrayList<Pose3d>();
    double[] visCornersX = new double[4 * result.getTargets().size()];
    double[] visCornersY = new double[4 * result.getTargets().size()];

    if (result.getTargets().size() < 2) {
      // Run fallback strategy instead
      return update(result, fieldTags, primaryStrategy, multiTagFallbackStrategy);
    }

    for (var target : result.getTargets()) {
      visCorners.addAll(target.getDetectedCorners());

      var tagPoseOpt = fieldTags.getTagPose(target.getFiducialId());
      if (tagPoseOpt.isEmpty()) {
        DriverStation.reportError(
            "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: "
                + target.getFiducialId(),
            false);
        continue;
      }

      var tagPose = tagPoseOpt.get();

      // actual layout poses of visible tags -- not exposed, so have to recreate
      knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));

      fieldToCams.add(tagPose.transformBy(target.getBestCameraToTarget().inverse()));
      fieldToCamsAlt.add(tagPose.transformBy(target.getAlternateCameraToTarget().inverse()));
    }

    boolean hasCalibData = true;

    for (TargetCorner corner : visCorners) {
      visCornersX[visCorners.indexOf(corner)] = corner.x;
      visCornersY[visCorners.indexOf(corner)] = corner.y;
    }
    // multi-target solvePNP
    if (hasCalibData) {
      var pnpResults =
          VisionEstimation.estimateCamPosePNP(
              CAMERA_MATRIX_OPT,
              DIST_COEFFS_OPT,
              result.targets,
              fieldTags,
              TargetModel.kAprilTag36h11);
      var best =
          new Pose3d()
              .plus(pnpResults.best) // field-to-camera
              .plus(CAMERA_TO_ROBOT); // field-to-robot
      // var alt = new Pose3d()
      // .plus(pnpResults.alt) // field-to-camera
      // .plus(robotToCamera.inverse()); // field-to-robot
      var estimatedRobotPose =
          new EstimatedRobotPose(
              best, result.getTimestampSeconds(), result.getTargets(), primaryStrategy);
      return Optional.of(estimatedRobotPose);
    } else {
      System.out.println("No calib data, fallback to lowest ambiguity");
      return lowestAmbiguityStrategy(result, fieldTags);
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
      PhotonPipelineResult result, AprilTagFieldLayout fieldTags) {
    PhotonTrackedTarget lowestAmbiguityTarget = null;

    double lowestAmbiguityScore = 10;

    for (PhotonTrackedTarget target : result.targets) {
      double targetPoseAmbiguity = target.getPoseAmbiguity();

      if (target.getFiducialId() < 1 || target.getFiducialId() > 8) continue;

      if (target.getBestCameraToTarget().getTranslation().getNorm() > 3) {
        continue;
      }

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

    Optional<Pose3d> targetPosition = fieldTags.getTagPose(targetFiducialId);

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
                .transformBy(CAMERA_TO_ROBOT),
            result.getTimestampSeconds(),
            result.getTargets(),
            PoseStrategy.LOWEST_AMBIGUITY);
    return Optional.of(estimatedRobotPose);
  }

  // 5026
  public static record UnitDeviationParams(
      double distanceMultiplier, double eulerMultiplier, double minimum) {
    private double computeUnitDeviation(double averageDistance) {
      return Math.max(minimum, eulerMultiplier * Math.exp(averageDistance * distanceMultiplier));
    }
  }

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

  public static Matrix<N3, N1> findVisionMeasurements(EstimatedRobotPose estimation) {
    double sumDistance = 0;
    for (var target : estimation.targetsUsed) {
      var t3d = target.getBestCameraToTarget();
      sumDistance +=
          Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
    }
    double avgDistance = sumDistance / estimation.targetsUsed.size();

    var deviation =
        TAG_COUNT_DEVIATION_PARAMS
            .get(
                MathUtil.clamp(
                    estimation.targetsUsed.size() - 1, 0, TAG_COUNT_DEVIATION_PARAMS.size() - 1))
            .computeDeviation(avgDistance);

    // System.out.println(
    //     String.format(
    //         "with %d tags at smallest distance %f and pose ambiguity factor %f, confidence
    // multiplier %f",
    //         estimation.targetsUsed.size(),
    //         smallestDistance,
    //         poseAmbiguityFactor,
    //         confidenceMultiplier));

    return deviation;
  }
  // Reject unreasonable vision poses
  public static void sanityCheck(VisionMeasurement measurement, EstimatedRobotPose estimatedPose) {
    while (measurement != null) { // Could change depending on what the climb is
      if (Math.abs(measurement.estimation.estimatedPose.getZ()) > 0.5) {
        continue;
      }
      // Skip single-tag measurements with too-high ambiguity.
      if (measurement.estimation.targetsUsed.size() < 2
          && measurement
                  .estimation
                  .targetsUsed
                  .get(0)
                  .getBestCameraToTarget()
                  .getTranslation()
                  .getNorm()
              > Units.feetToMeters(13)) {
        continue;
      }
    }
  }
}