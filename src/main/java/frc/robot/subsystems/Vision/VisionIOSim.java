// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

/** Add your docs here. */
public class VisionIOSim implements VisionIO {
  public static final String SIM_VISION_SYSTEM_NAME = "sim vision";
  public static final String SIM_CAMERA_NAME = "sim camera";
  VisionSystemSim sim = new VisionSystemSim(SIM_VISION_SYSTEM_NAME);
  PhotonCamera simCamera = new PhotonCamera(SIM_CAMERA_NAME);

  public VisionIOSim() {
    try {
      var field =
          AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(); // TODO change to crescendo
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      sim.addVisionTargets(
          new VisionTargetSim(new Pose3d(), new TargetModel(null))); // TODO change to crescendo
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d pose) {
    var result = simCamera.getLatestResult();
    sim.update(pose);
    inputs.timestamp = result.getTimestampSeconds();
    inputs.latency = result.getLatencyMillis();
    // inputs.targets = result.targets; //TODO
  }
}
