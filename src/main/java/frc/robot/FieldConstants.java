// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.AutoLogOutput;

/** Add your docs here. */
public class FieldConstants {

  public static final Pose2d BLUE_SPEAKER_POSE =
      new Pose2d(new Translation2d(-0.0381, 5.547868), Rotation2d.fromDegrees(180));
  public static final Pose2d RED_SPEAKER_POSE =
      new Pose2d(new Translation2d(16.579342, 5.547868), Rotation2d.fromDegrees(0));

  public static final Translation2d BLUE_AMP_POSE = new Translation2d(2.182, 7.469);
  public static final Translation2d RED_AMP_POSE = new Translation2d(14.277, 7.432);

  public static Translation2d getAmp() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get() == Alliance.Red ? RED_AMP_POSE : BLUE_AMP_POSE;
    } else {
      return BLUE_AMP_POSE; // default to blue
    }
  }

  @AutoLogOutput(key = "AutoAim/Speaker")
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
