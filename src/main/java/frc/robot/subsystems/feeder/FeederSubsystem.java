// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Feeder motor for shooter and associated beambreaks for indexing */
public class FeederSubsystem extends SubsystemBase {
  public static final double INDEXING_VOLTAGE = 4.0;
  public static final double INDEXING_VELOCITY = 35.0;

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  /** Run the feeder at a set voltage */
  public Command runVoltageCmd(double volts) {
    return this.run(() -> io.setVoltage(volts));
  }

  public Command runVelocityCmd(double velocity) {
    return this.run(() -> io.setVelocity(velocity));
  }

  /** Run the feeder to place the ring between the beambreaks. */
  public Command indexCmd() {
    // return this.run(
    //     () -> {
    //       if (inputs.lastBeambreak) {
    //         io.setVoltage(-INDEXING_VOLTAGE);
    //       } else if (inputs.firstBeambreak) {
    //         io.setVoltage(0.0);
    //       } else {
    //         io.setVoltage(INDEXING_VOLTAGE);
    //       }
    //     });
    return this.run(
        () -> {
          if (inputs.lastBeambreak) {
            io.setVelocity(-INDEXING_VELOCITY);
          } else if (inputs.firstBeambreak) {
            io.setVelocity(0.0);
          } else {
            io.setVelocity(INDEXING_VELOCITY);
          }
        });
  }

  public boolean getFirstBeambreak() {
    return inputs.firstBeambreak;
  }
}
