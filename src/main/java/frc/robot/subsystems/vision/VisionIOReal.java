// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.Vision.VisionConstants;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {
  // constants
  private PhotonCamera camera;
  private VisionConstants constants;

  public VisionIOReal(VisionConstants constants) {
    this.constants = constants;
    camera = new PhotonCamera(constants.cameraName());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var result = camera.getLatestResult();
    inputs.timestamp = result.getTimestampSeconds();
    inputs.latency = result.getLatencyMillis();
    inputs.targets = result.targets;
    inputs.numTags = result.targets.size();
    inputs.constants = constants;
  }
}
