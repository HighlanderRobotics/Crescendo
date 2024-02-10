// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.reaction_bar_release;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReactionBarReleaseSubsystem extends SubsystemBase {
  private final ReactionBarReleaseIO io;
  private final ReactionBarReleaseIOInputsAutoLogged inputs = new ReactionBarReleaseIOInputsAutoLogged();
  /** Creates a new ReactionBarReleaseSubsystem. */
  public ReactionBarReleaseSubsystem(ReactionBarReleaseIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Reaction Bar Release", inputs);
  }

  public Command setRotationCmd(final Rotation2d rotation) {
    return this.run(() -> setRotationCmd(rotation));
  }
}
