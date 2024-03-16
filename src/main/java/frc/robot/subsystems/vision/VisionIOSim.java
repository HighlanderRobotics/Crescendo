// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Add your docs here. */
public class VisionIOSim implements VisionIO {
  private final VisionSystemSim sim;
  private final PhotonCamera camera;
  private final PhotonCameraSim simCamera;
  private final VisionConstants constants;
  public static Supplier<Pose3d> pose;

  public VisionIOSim(VisionConstants constants) {
    this.sim = new VisionSystemSim(constants.cameraName());
    var cameraProp = new SimCameraProperties();
    // TODO Fix these constants
    cameraProp.setCalibration(1080, 960, constants.intrinsicsMatrix(), constants.distCoeffs());
    cameraProp.setCalibError(0.0, 0.0);
    cameraProp.setFPS(50.0);
    cameraProp.setAvgLatencyMs(30.0);
    cameraProp.setLatencyStdDevMs(5.0);
    this.camera = new PhotonCamera(constants.cameraName());
    this.simCamera = new PhotonCameraSim(camera, cameraProp);
    simCamera.enableDrawWireframe(true);
    simCamera.setMaxSightRange(7.0);
    this.constants = constants;
    sim.addCamera(simCamera, constants.robotToCamera());

    try {
      var field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      sim.addAprilTags(field);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void setSimPose(Optional<EstimatedRobotPose> simEst, Vision camera, boolean newResult) {
    simEst.ifPresentOrElse(
        est ->
            VisionHelper.getSimDebugField(sim)
                .getObject("VisionEstimation")
                .setPose(est.estimatedPose.toPose2d()),
        () -> {
          if (newResult)
            VisionHelper.getSimDebugField(sim).getObject("VisionEstimation").setPoses();
        });
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var result = camera.getLatestResult();
    sim.update(pose.get().toPose2d());
    inputs.timestamp = result.getTimestampSeconds();
    inputs.latency = result.getLatencyMillis();
    inputs.targets = result.targets; // TODO aaaaaaa
    inputs.constants = constants;
  }

  @Override
  public String getName() {
    return constants.cameraName();
  }
}
