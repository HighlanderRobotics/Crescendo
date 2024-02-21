// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public interface VisionIO {

  public static class VisionIOInputs {
    public double timestamp = 0.0;
    public double latency = 0.0;
    public List<PhotonTrackedTarget> targets =
        new ArrayList<>(); // TODO make protobuf work whenever that happens
    public double numTags = 0; // TODO why isn't this just targets.size()?
    public Pose3d coprocPNPPose = new Pose3d();
    public Pose3d[] targetPose3ds;
    public VisionConstants constants;
  }

  public void updateInputs(VisionIOInputs inputs);

  public void setSimPose(
      Optional<EstimatedRobotPose> simEst, Vision camera, boolean newResult);

  public String getName();
}
