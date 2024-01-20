// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {
  // constants
  public static String CAMERA_NAME;
  public static PhotonCamera camera;

  /*** Transform3d from the center of the robot to the camera mount position (ie,
   *     robot ➔ camera) in the <a href=
   *     "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *     Coordinate System</a>.
   ***/
  public Transform3d robotToCamera;

  public VisionIOReal(VisionConstants constants) {
    CAMERA_NAME = constants.cameraName();
    camera = new PhotonCamera(CAMERA_NAME);
    robotToCamera = constants.robotToCamera();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d pose) { // TODO
    var result = camera.getLatestResult();
    inputs.timestamp = result.getTimestampSeconds();
    inputs.latency = result.getLatencyMillis();
    inputs.coprocPNPTargets = result.targets;
    inputs.numTags = result.targets.size();
    inputs.pose = pose;
  }

  public void updateInputs() {}
}
