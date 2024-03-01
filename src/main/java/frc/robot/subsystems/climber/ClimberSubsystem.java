// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  public static final Rotation2d CLIMBER_MIN_ANGLE = Rotation2d.fromDegrees(0.0); //TODO
  public static final Rotation2d CLIMBER_MAX_ANGLE = Rotation2d.fromDegrees(0.0); //TODO
  public static final Rotation2d CLIMB_ANGLE = Rotation2d.fromDegrees(0.0); //TODO

  final ClimberIO io;
  final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command runVoltageCmd(double voltage) {
    return this.run(() -> io.setClimberVoltage(voltage));
  }

  public Command extendClimb() {
    return this.run(
      () -> {
        io.setSetpoint(CLIMB_ANGLE); //TODO find
      }
    );
  }

  public Command retractClimb() {
    return this.run(
      () -> {
        io.setSetpoint(CLIMBER_MIN_ANGLE); //TODO find
      }
    );
  }

  public Command runClimberCurrentZeroing() { //TODO numbers are from shooter pivot
    return this.run(() -> io.setClimberVoltage(-1.0))
        .until(() -> inputs.climberCurrentAmps > 40.0)
        .finallyDo(() -> io.resetPosition(CLIMBER_MIN_ANGLE));
  }
  public Rotation2d getPosition() {
    return inputs.climberRotation;
  }
}
