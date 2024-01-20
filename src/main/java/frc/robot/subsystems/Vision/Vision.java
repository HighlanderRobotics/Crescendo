// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

/** Add your docs here. */
public class Vision {
  public record VisionConstants(String cameraName, Transform3d robotToCamera) {}

  public final Matrix<N3, N3> CAMERA_MATRIX_OPT =
      MatBuilder.fill(
          Nat.N3(),
          Nat.N3(),
          923.5403619629557,
          0.0,
          644.4965658066068,
          0.0,
          925.8136962361125,
          402.6412935350414,
          0.0,
          0.0,
          1.0); // TODO tune maybe
  public final Matrix<N5, N1> DIST_COEFFS_OPT =
      MatBuilder.fill(
          Nat.N5(),
          Nat.N1(),
          0.05452153950284706,
          -0.04331612051891956,
          0.00176988756858703,
          -0.004530368741385627,
          -0.040501622476628085); // TODO tune maybe

  public final VisionIO io; // maybe this shouldnt be public idk
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
