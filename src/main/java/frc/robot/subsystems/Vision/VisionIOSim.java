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

/** Add your docs here. */
public class VisionIOSim implements VisionIO {
  public static final String SIM_VISION_SYSTEM_NAME = "sim vision";
  public static final String SIM_CAMERA_NAME = "sim camera";
  VisionSystemSim sim = new VisionSystemSim(SIM_VISION_SYSTEM_NAME);
  PhotonCamera camera = new PhotonCamera(SIM_CAMERA_NAME); //TODO ???? what how why
  SimCameraProperties cameraProp = new SimCameraProperties(); //TODO find
  PhotonCameraSim simCamera = new PhotonCameraSim(camera, cameraProp);

  public VisionIOSim() {
    try {
      var field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      sim.addAprilTags(field);
      sim.addCamera(simCamera, VisionHelper.CAMERA_TO_ROBOT);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d pose) {
    // var result = 
    // sim.update(pose);
    // inputs.timestamp = result.getTimestampSeconds();
    // inputs.latency = result.getLatencyMillis();
    // inputs.targets = result.targets; //TODO aaaaaaa
  }
}
