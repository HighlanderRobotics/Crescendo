// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  //TODO check
  public static final double CLIMBER_MIN_ROTATIONS = 0.0;
  public static final double CLIMBER_MAX_ROTATIONS = 3.0;
  public static final double CLIMB_ROTATIONS = 2.9;
  public static final double SENSOR_TO_MECHANISM_RATIO = 64.0;

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

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

  public Command extendRotationsCmd(double rotations) {
    return this.run(
        () -> {
          io.setSetpoint(rotations);
        });
  }

  public Command extendClimbCmd() {
    return extendRotationsCmd(CLIMB_ROTATIONS);
  }

  public Command retractClimbCmd() {
    return extendRotationsCmd(CLIMBER_MIN_ROTATIONS);
  }

  public Command runClimberCurrentZeroing() {
    return this.run(() -> io.setClimberVoltage(-1.0))
        .until(() -> inputs.climberCurrentAmps > 40.0)
        .finallyDo(() -> io.resetPosition(CLIMBER_MIN_ROTATIONS));
  }

  public double getRotations() {
    return inputs.climberRotations;
  }
}
