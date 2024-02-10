// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.reaction_bar_release;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;

/** Reaction Bar IO with a real servo. */
public class ReactionBarReleaseIOReal implements ReactionBarReleaseIO {
  Servo servo = new Servo(0);

  @Override
  public void updateInputs(ReactionBarReleaseIOInputs inputs) {
    // no-op bc inputs is empty
  }

  @Override
  public void setRotation(Rotation2d rotation) {
    servo.set(rotation.getRotations());
  }
}
