// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Add your docs here. */
public class VisionIOSim implements VisionIO {
  public String simCameraName;
  VisionSystemSim sim;
  PhotonCamera camera;
  SimCameraProperties cameraProp;
  PhotonCameraSim simCamera;
  Transform3d robotToCamera;
  Supplier<Pose3d> pose;

  public VisionIOSim(VisionConstants constants, Supplier<Pose3d> pose) {
    this.simCameraName = constants.cameraName();
    this.sim = constants.simSystem();
    this.cameraProp = new SimCameraProperties();
    this.camera = new PhotonCamera(simCameraName);
    this.simCamera = new PhotonCameraSim(camera, cameraProp);
    this.robotToCamera = constants.robotToCamera();
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
  public void updateInputs(VisionIOInputs inputs) {
    var result = camera.getLatestResult();
    sim.update(pose.get().toPose2d());
    inputs.timestamp = result.getTimestampSeconds();
    inputs.latency = result.getLatencyMillis();
    inputs.targets = result.targets; // TODO aaaaaaa
  }
}
