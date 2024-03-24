// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class Vision {
  public static final Matrix<N3, N1> visionPointBlankDevs =
      new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {0.4, 0.4, 1});
  public static final double distanceFactor = 0.5;

  public record VisionConstants(
      String cameraName,
      Transform3d robotToCamera,
      Matrix<N3, N3> intrinsicsMatrix,
      Matrix<N5, N1> distCoeffs) {}

  private final VisionIO io;
  public final VisionIOInputsLogged inputs = new VisionIOInputsLogged();

  public Vision(final VisionIO io) {
    this.io = io;
  }

  public void setSimPose(Optional<EstimatedRobotPose> simEst, Vision camera, boolean newResult) {
    this.io.setSimPose(simEst, camera, newResult);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void processInputs() {
    Logger.processInputs("Apriltag Vision/" + io.getName(), inputs);
  }

  public Optional<EstimatedRobotPose> update(PhotonPipelineResult result) {
    // Skip if we only have 1 target
    // TODO change
    if (result.getTargets().size() < 1) {
      return Optional.empty();
    }
    var estPose =
        VisionHelper.update(
            result,
            inputs.constants.intrinsicsMatrix(),
            inputs.constants.distCoeffs(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            inputs.constants.robotToCamera(),
            inputs.coprocPNPTransform);
    // // Reject if estimated pose is in the air or ground
    if (estPose.isPresent() && Math.abs(estPose.get().estimatedPose.getZ()) > 0.25) {
      return Optional.empty();
    }
    return estPose;
  }

  public String getName() {
    return io.getName();
  }
}
