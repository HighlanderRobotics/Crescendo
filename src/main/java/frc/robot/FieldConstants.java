// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldConstants {

  public static final Pose2d BLUE_SPEAKER_POSE =
      new Pose2d(new Translation2d(-0.086473, 5.4), Rotation2d.fromDegrees(0));
  public static final Pose2d RED_SPEAKER_POSE =
      new Pose2d(new Translation2d(16.389722, 5.4), Rotation2d.fromDegrees(180));

  public static Pose2d getSpeaker() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get() == Alliance.Red
          ? RED_SPEAKER_POSE
          : BLUE_SPEAKER_POSE;
    } else {
      return BLUE_SPEAKER_POSE; // default to blue
    }
  }
}
