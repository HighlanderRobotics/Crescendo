// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.carriage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Tracer;
import org.littletonrobotics.junction.Logger;

/** Drainpipe style amp/trap mechanism on the elevator */
public class CarriageSubsystem extends SubsystemBase {
  public static final double INDEXING_VOLTAGE = 3.0;

  final CarriageIO io;
  final CarriageIOInputsAutoLogged inputs = new CarriageIOInputsAutoLogged();

  /** Creates a new CarriageSubsystem. */
  public CarriageSubsystem(CarriageIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    Tracer.startTrace("CarriagePeriodic");
    io.updateInputs(inputs);
    Logger.processInputs("Carriage", inputs);
    Tracer.endTrace();
  }

  /** Run the carriage roller at the specified voltage */
  public Command runVoltageCmd(double voltage) {
    return this.run(() -> io.setVoltage(voltage));
  }

  /**
   * Run the amp mech forward until the note is triggering the beambreak, then jog it a little
   * further forward.
   */
  public Command indexForwardsCmd() {
    return runVoltageCmd(INDEXING_VOLTAGE)
        .until(() -> inputs.beambreak)
        .andThen(runVoltageCmd(INDEXING_VOLTAGE / 2).withTimeout(0.12), runVoltageCmd(0.0))
        .until(() -> !getBeambreak())
        .repeatedly();
  }

  /** Run the amp mech backwards until the beambreak cycles, then forward index. */
  public Command indexBackwardsCmd() {
    return Commands.sequence(
        runVoltageCmd(-INDEXING_VOLTAGE).until(() -> inputs.beambreak),
        runVoltageCmd(-INDEXING_VOLTAGE).withTimeout(0.1),
        runVoltageCmd(-INDEXING_VOLTAGE).until(() -> !inputs.beambreak),
        runVoltageCmd(-INDEXING_VOLTAGE).withTimeout(0.1),
        indexForwardsCmd());
  }

  public boolean getBeambreak() {
    return inputs.beambreak;
  }
}
