// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.carriage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

/** Drainpipe style amp/trap mechanism on the elevator */
public class CarriageSubsystem extends SubsystemBase {
  public static final double INDEXING_VOLTAGE = 3.0;

  final CarriageIO io;
  final CarriageIOInputsAutoLogged inputs = new CarriageIOInputsAutoLogged();

  private boolean isIndexed;
  public final Trigger isIndexedTrig = new Trigger(() -> isIndexed);
  public final Trigger beambreakTrig = new Trigger(this::getBeambreak);

  /** Creates a new CarriageSubsystem. */
  public CarriageSubsystem(CarriageIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Carriage", inputs);
  }

  /** Run the carriage roller at the specified voltage */
  public Command setVoltageCmd(double voltage) {
    return this.run(() -> io.setVoltage(voltage));
  }

  public Command stop() {
    return this.setVoltageCmd(0.0);
  }

  public Command setIndexingVoltageCmd() {
    return setVoltageCmd(INDEXING_VOLTAGE);
  }

  /**
   * Run the amp mech forward until the note is triggering the beambreak, then jog it a little
   * further forward.
   */
  public Command indexForwardsCmd() {
    return setVoltageCmd(INDEXING_VOLTAGE).beforeStarting(() -> isIndexed = false)
        .until(() -> inputs.beambreak)
        .andThen(setVoltageCmd(INDEXING_VOLTAGE / 2).withTimeout(0.12), setVoltageCmd(0.0).finallyDo(() -> isIndexed = true))
        .until(() -> !getBeambreak())
        .repeatedly();
  }

  /** Run the amp mech backwards until the beambreak cycles, then forward index. */
  public Command indexBackwardsCmd() {
    return Commands.sequence(
        setVoltageCmd(-INDEXING_VOLTAGE).until(() -> inputs.beambreak),
        setVoltageCmd(-INDEXING_VOLTAGE).withTimeout(0.1),
        setVoltageCmd(-INDEXING_VOLTAGE).until(() -> !inputs.beambreak),
        setVoltageCmd(-INDEXING_VOLTAGE).withTimeout(0.1),
        indexForwardsCmd());
  }

  public boolean getBeambreak() {
    return inputs.beambreak;
  }
}
