// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;

/** Add your docs here. */
public class AutoAim {

  public static final InterpolatingShotTree shotMap = new InterpolatingShotTree();

  public static final double LOOKAHEAD_TIME_SECONDS = 1.0;

  static {
    shotMap.put(
        1.0 + Units.inchesToMeters(13.5 + 3.25),
        new ShotData(Rotation2d.fromDegrees(55), 60, 80, 0.25));
    shotMap.put(
        2.0 + Units.inchesToMeters(13.5 + 3.25),
        new ShotData(Rotation2d.fromDegrees(44), 60, 80, 0.31));
    shotMap.put(
        3.0 + Units.inchesToMeters(13.5 + 3.25),
        new ShotData(Rotation2d.fromDegrees(34), 60, 80, 0.36));
    shotMap.put(
        4.0 + Units.inchesToMeters(13.5 + 3.25),
        new ShotData(Rotation2d.fromDegrees(30), 60, 100, 0.4));
    shotMap.put(
        5.0 + Units.inchesToMeters(13.5 + 3.25),
        new ShotData(Rotation2d.fromDegrees(27), 50, 100, 0.45)); // TODO check
  }

  /**
   * Transforms the speaker pose by the robots current velocity (assumes constant velocity)
   *
   * @return The transformed pose
   */
  public static Pose2d getVirtualTarget(Pose2d pose, ChassisSpeeds speedsFieldRelative) {
    Pose2d target = FieldConstants.getSpeaker();

    double distance =
            pose
            .minus(target)
            .getTranslation()
            .getNorm();

    return target.transformBy(
        new Transform2d(
                speedsFieldRelative.vxMetersPerSecond
                    * AutoAim.shotMap.get(distance).getFlightTimeSeconds(),
                speedsFieldRelative.vyMetersPerSecond
                    * AutoAim.shotMap.get(distance).getFlightTimeSeconds(),
                new Rotation2d())
            .inverse());
  }
}
