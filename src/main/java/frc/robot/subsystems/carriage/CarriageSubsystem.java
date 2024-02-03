// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.carriage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CarriageSubsystem extends SubsystemBase {
  CarriageIO io;
  CarriageIOInputsAutoLogged inputs = new CarriageIOInputsAutoLogged();

  /** Creates a new IntakeSubsystem. */
  public CarriageSubsystem(CarriageIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command runVoltageCmd(double voltage) {
    return this.run(() -> io.setVoltage(voltage));
  }

  /** Run the amp mech until the note is triggering the beambreak */
  public Command index() {
    return this.run(() -> runVoltageCmd(3.0)).until(() -> inputs.beambreak);
  }
}
