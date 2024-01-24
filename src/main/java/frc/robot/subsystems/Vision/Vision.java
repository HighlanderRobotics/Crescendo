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

/** Add your docs here. */
public class Vision {
  public record VisionConstants(
      String cameraName,
      Transform3d robotToCamera,
      String simVisionSystemName,
      Matrix<N3, N3> CAMERA_MATRIX_OPT,
      Matrix<N5, N1> DIST_COEFFS_OPT) {}

  private final VisionIO io;
  public final VisionIOInputsLogged inputs = new VisionIOInputsLogged();
  public final VisionConstants constants;

  public Vision(final VisionIO io, final VisionConstants constants) {
    this.io = io;
    this.constants = constants;
  }

  public void updateInputs(Pose3d pose) {
    io.updateInputs(inputs, pose);
  }
}
