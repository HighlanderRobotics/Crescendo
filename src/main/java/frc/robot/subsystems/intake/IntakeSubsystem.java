// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** 95 style utb intake */
public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** Run the intake and centering motors at the specified voltage */
  public Command runVoltageCmd(double intakeVoltage, double centeringVoltage) {
    return this.run(
        () -> {
          io.setIntakeVoltage(intakeVoltage);
          io.setCenteringVoltage(centeringVoltage);
        });
  }

  public Command runVelocityCmd(double intakeVelocity, double centeringVelocity) {
    return this.run(
        () -> {
          io.setIntakeSpeed(intakeVelocity);
          io.setCenteringSpeed(centeringVelocity);
        });
  }
}
