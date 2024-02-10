// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.reaction_bar_release;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ReactionBarReleaseIO {
  @AutoLog
  public static class ReactionBarReleaseIOInputs {
    // Nothing here since no sensors
  }

  public void updateInputs(final ReactionBarReleaseIOInputs inputs);

  public void setRotation(final Rotation2d rotation);
}
