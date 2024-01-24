// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {
  // constants
  public String cameraName;
  public PhotonCamera camera;
  public Matrix<N3, N3> cameraMatrixOpt;
  public Matrix<N5, N1> distCoeffsOpt;

  /*** Transform3d from the center of the robot to the camera mount position (ie,
   *     robot ➔ camera) in the <a href=
   *     "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *     Coordinate System</a>.
   ***/
  public Transform3d robotToCamera;

  public VisionIOReal(VisionConstants constants) {
    cameraName = constants.cameraName();
    camera = new PhotonCamera(cameraName);
    robotToCamera = constants.robotToCamera();
    cameraMatrixOpt = constants.CAMERA_MATRIX_OPT();
    distCoeffsOpt = constants.DIST_COEFFS_OPT();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d pose) {
    var result = camera.getLatestResult();
    inputs.timestamp = result.getTimestampSeconds();
    inputs.latency = result.getLatencyMillis();
    inputs.targets = result.targets;
    inputs.numTags = result.targets.size();
    inputs.coprocPNPPose = pose;
  }
}
