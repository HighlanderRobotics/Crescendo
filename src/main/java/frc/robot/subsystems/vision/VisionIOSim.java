// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class VisionIOSim implements VisionIO {
  public String simCameraName;
  VisionSystemSim sim;
  PhotonCamera camera;
  SimCameraProperties cameraProp;
  PhotonCameraSim simCamera;
  Transform3d robotToCamera;
  public static Supplier<Pose3d> pose;

  public Matrix<N3, N3> cameraMatrix;
  public Matrix<N5, N1> distCoeffs;

  public VisionIOSim(VisionConstants constants) {
    this.simCameraName = constants.cameraName();
    this.sim = constants.simSystem();
    this.cameraProp = new SimCameraProperties();
    this.camera = new PhotonCamera(simCameraName);
    this.simCamera = new PhotonCameraSim(camera, cameraProp);
    this.robotToCamera = constants.robotToCamera();
    this.cameraMatrix = constants.cameraMatrix();
    this.distCoeffs = constants.distCoeffs();
    sim.addCamera(simCamera, robotToCamera);

    // TODO find
    // cameraProp.setCalibration(kResolutionWidth, kResolutionHeight, kFOVDiag);
    // cameraProp.setCalibError(kAvgErrorPx, kErrorStdDevPx);
    // cameraProp.setFPS(kFPS);
    // cameraProp.setAvgLatencyMs(kAvgLatencyMs);
    // cameraProp.setLatencyStdDevMs(kLatencyStdDevMs);

    try {
      var field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      sim.addAprilTags(field);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void setSimPose(Optional<EstimatedRobotPose> simEst, Vision camera, boolean newResult) {
    simEst.ifPresentOrElse(
        est ->
            VisionHelper.getSimDebugField(sim)
                .getObject("VisionEstimation")
                .setPose(est.estimatedPose.toPose2d()),
        () -> {
          if (newResult)
            VisionHelper.getSimDebugField(sim).getObject("VisionEstimation").setPoses();
        });
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var result = camera.getLatestResult();
    sim.update(pose.get().toPose2d());
    inputs.timestamp = result.getTimestampSeconds();
    inputs.latency = result.getLatencyMillis();
    inputs.targets = result.targets; // TODO aaaaaaa
  }

  @Override
  public String getName() {
    return simCameraName;
  }

  @Override
  public Optional<EstimatedRobotPose> update(PhotonPipelineResult result) {
    var estPose =
        VisionHelper.update(
            result,
            cameraMatrix,
            distCoeffs,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera);
    return estPose;
  }
}
