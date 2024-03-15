// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldConstants {

  public static final Translation2d BLUE_SPEAKER_POSE = new Translation2d(-0.086473, 5.757474);
  public static final Translation2d RED_SPEAKER_POSE = new Translation2d(16.389722, 5.757474);

  public static final Translation2d BLUE_AMP_POSE = new Translation2d(-0.086473, 5.757474);
  public static final Translation2d RED_AMP_POSE = new Translation2d(16.389722, 5.757474);

  public static Translation2d getAmp() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get() == Alliance.Red ? RED_AMP_POSE : BLUE_AMP_POSE;
    } else {
      return BLUE_AMP_POSE; // default to blue
    }
  }

  public static Translation2d getSpeaker() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get() == Alliance.Red
          ? RED_SPEAKER_POSE
          : BLUE_SPEAKER_POSE;
    } else {
      return BLUE_SPEAKER_POSE; // default to blue
    }
  }
}
