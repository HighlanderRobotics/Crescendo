// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {
  // constants
  public static final String CAMERA_NAME = "opi-camera";
  public static PhotonCamera camera = new PhotonCamera(CAMERA_NAME);

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d pose) {
    var latestResult = camera.getLatestResult();
    inputs.timestamp = latestResult.getTimestampSeconds();
    inputs.latency = latestResult.getLatencyMillis();
    // inputs.targets = latestResult.targets; //TODO
  }
}
