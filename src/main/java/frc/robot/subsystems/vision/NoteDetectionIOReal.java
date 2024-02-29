// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/** Add your docs here. */
public class NoteDetectionIOReal implements NoteDetectionIO {
  public String cameraName;
  public PhotonCamera camera;
  public Matrix<N3, N3> cameraMatrix;
  public Matrix<N5, N1> distCoeffs;

  private final VisionConstants constants;

  public NoteDetectionIOReal(VisionConstants constants) {
    cameraName = constants.cameraName();
    camera = new PhotonCamera(cameraName);
    cameraMatrix = constants.intrinsicsMatrix();
    distCoeffs = constants.distCoeffs();
    this.constants = constants;
  }

  @Override
  public void updateInputs(NoteDetectionIOInputs inputs) {
    var result = camera.getLatestResult();
    inputs.timestamp = result.getTimestampSeconds();
    inputs.latency = result.getLatencyMillis();
    inputs.targets = result.targets;
    inputs.constants = constants;
    inputs.yaw = -Units.degreesToRadians(result.getBestTarget().getYaw()); // why is this negative
    inputs.pitch = -Units.degreesToRadians(result.getBestTarget().getPitch());
    inputs.distance =
        PhotonUtils.calculateDistanceToTargetMeters(
            0, 0, 0, Units.degreesToRadians(result.getBestTarget().getPitch())); // TODO fix
  }

  @Override
  public String getName() {
    return cameraName;
  }
}
