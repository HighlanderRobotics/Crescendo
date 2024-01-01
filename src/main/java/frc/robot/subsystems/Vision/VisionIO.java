// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public double timestamp = 0.0;
    public double latency = 0.0;
    // public List<PhotonTrackedTarget> targets = new ArrayList<>(); //TODO waiting on pv to add
    // protobuf support
    // public double numTags = 0;
    public Pose3d pose = new Pose3d();
  }

  public default void updateInputs(VisionIOInputs inputs, Pose3d pose) {}
  ;
}
