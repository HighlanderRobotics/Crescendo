// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public interface VisionIO {

  public static class VisionIOInputs {
    public double timestamp = 0.0;
    public double latency = 0.0;
    public List<PhotonTrackedTarget> coprocPNPTargets =
        new ArrayList<>(); // TODO make protobuf work whenever that happens
    public double numTags = 0; // TODO why isn't this just targets.size()?
    public Pose3d pose = new Pose3d();
  }

  public default void updateInputs(VisionIOInputs inputs, Pose3d pose) {}
}
