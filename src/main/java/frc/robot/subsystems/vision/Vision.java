// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class Vision {
  public record VisionConstants(
      String cameraName,
      Transform3d robotToCamera,
      VisionSystemSim simSystem,
      Matrix<N3, N3> cameraMatrix,
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
    Logger.processInputs(io.getName(), inputs);
  }

  public Optional<EstimatedRobotPose> update(PhotonPipelineResult result) {
    var estPose = io.update(result);
    return estPose;
  }

  public String getName() {
    return io.getName();
  }
}
