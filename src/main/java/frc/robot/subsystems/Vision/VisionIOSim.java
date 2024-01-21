// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Vision.VisionConstants;

/** Add your docs here. */
public class VisionIOSim implements VisionIO {
  public String simVisionSystemName;
  public String simCameraName;
  VisionSystemSim sim;
  PhotonCamera camera;
  SimCameraProperties cameraProp = new SimCameraProperties();
  PhotonCameraSim simCamera = new PhotonCameraSim(camera, cameraProp);
  Transform3d robotToCamera;

  public VisionIOSim(VisionConstants constants) {
    this.simCameraName = constants.cameraName();
    this.simVisionSystemName = constants.simVisionSystemName();
    this.sim = new VisionSystemSim(simVisionSystemName);
    this.camera = new PhotonCamera(simCameraName);
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
  public void updateInputs(VisionIOInputs inputs, Pose3d pose) {
    var result = camera.getLatestResult();
    sim.update(pose);
    inputs.timestamp = result.getTimestampSeconds();
    inputs.latency = result.getLatencyMillis();
    inputs.targets = result.targets; // TODO aaaaaaa
  }
}
