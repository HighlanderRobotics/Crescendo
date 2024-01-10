// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class PivotSubsystem extends SubsystemBase {
  PivotIO io;
  PivotIOInputsAutoLogged inputs;

  public PivotSubsystem(PivotIO io) {
    this.io = io;
    inputs = new PivotIOInputsAutoLogged();

    SmartDashboard.putData(
        "reset Pivot value",
        new InstantCommand(
            () -> {
              io.reset(0);
            }));
  }

  public Command reset() {
    return this.runOnce(
        () -> {
          io.reset(0);
        });
  }

  public Command run(double degrees) {
    return new RunCommand(
        () -> {
          io.setPosition(degrees);
        },
        this);
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();
    Logger.processInputs("Pivot", inputs);
  }
}
