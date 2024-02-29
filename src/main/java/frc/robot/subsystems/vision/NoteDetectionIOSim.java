// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class NoteDetectionIOSim implements NoteDetectionIO {
  public String cameraName;
  public PhotonCamera camera;
  public Matrix<N3, N3> cameraMatrix;
  public Matrix<N5, N1> distCoeffs;

  private final VisionConstants constants;

  public NoteDetectionIOSim(VisionConstants constants) {
    cameraName = constants.cameraName();
    camera = new PhotonCamera(cameraName);
    cameraMatrix = constants.intrinsicsMatrix();
    distCoeffs = constants.distCoeffs();
    this.constants = constants;
  }

  @Override
  public void updateInputs(NoteDetectionIOInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public String getName() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getName'");
  }
}
