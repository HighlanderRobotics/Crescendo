// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    public Transform3d coprocPNPTransform = new Transform3d();
    public Pose3d[] targetPose3ds = new Pose3d[] {};
    public VisionConstants constants =
        new VisionConstants(
            "Default",
            new Transform3d(),
            Matrix.eye(Nat.N3()),
            MatBuilder.fill(Nat.N5(), Nat.N1(), 0.0, 0.0, 0.0, 0.0, 0.0));
  }

  public void updateInputs(VisionIOInputs inputs);

  public void setSimPose(Optional<EstimatedRobotPose> simEst, Vision camera, boolean newResult);

  public String getName();
}
