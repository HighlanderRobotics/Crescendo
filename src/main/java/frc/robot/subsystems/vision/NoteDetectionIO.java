// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.Vision.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public interface NoteDetectionIO { //TODO make vision superinterface/class?

  public static class NoteDetectionIOInputs {
    public double timestamp = 0.0;
    public double latency = 0.0;
    public List<PhotonTrackedTarget> targets = new ArrayList<>(); // TODO protobuf
    public double numTargets = 0;
    public VisionConstants constants;
    public double yaw = 0.0;
    public double pitch = 0.0;
    public double distance = 0.0;
  }

  public void updateInputs(NoteDetectionIOInputs inputs);

  public String getName();
}
