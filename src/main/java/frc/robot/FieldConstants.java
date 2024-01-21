// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class FieldConstants {

  public static final Pose2d BLUE_SPEAKER_POSE =
      new Pose2d(new Translation2d(-0.086473, 5.757474), new Rotation2d());
  public static final Pose2d RED_SPEAKER_POSE =
      new Pose2d(new Translation2d(16.389722, 5.757474), new Rotation2d());

  public static final Pose2d BLUE_AMP_POSE =
      new Pose2d(new Translation2d(1.932627, 8.531136), new Rotation2d());

  public static final Pose2d RED_AMP_POSE =
      new Pose2d(new Translation2d(14.738367, 8.505067), new Rotation2d());

  public static Pose2d getAmp() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get() == Alliance.Red ? RED_AMP_POSE : BLUE_AMP_POSE;
    } else {
      return BLUE_AMP_POSE; // default to blue
    }
  }

  public static Pose2d getSpeaker() {
    Logger.recordOutput("blue speaker", BLUE_SPEAKER_POSE);
    Logger.recordOutput("red speaker", RED_SPEAKER_POSE);
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get() == Alliance.Red
          ? RED_SPEAKER_POSE
          : BLUE_SPEAKER_POSE;
    } else {
      return BLUE_SPEAKER_POSE; // default to blue
    }
  }
}
