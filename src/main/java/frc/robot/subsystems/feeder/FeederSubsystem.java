// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Feeder motor for shooter and associated beambreaks for indexing */
public class FeederSubsystem extends SubsystemBase {
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

  /** Run the feeder to place the ring between the beambreaks. */
  public Command indexCmd() {
    return this.run(
        () -> {
          if (inputs.lastBeambreak) {
            io.setVoltage(-3.0);
          } else if (inputs.firstBeambreak) {
            io.setVoltage(0.0);
          } else {
            io.setVoltage(3.0);
          }
        });
  }

  public boolean getFirstBeambreak() {
    return inputs.firstBeambreak;
  }
}
