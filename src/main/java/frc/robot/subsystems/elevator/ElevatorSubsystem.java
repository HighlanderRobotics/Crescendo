// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  ElevatorIO io;

  // For dashboard
  Mechanism2d mech2d = new Mechanism2d(3.0, Units.feetToMeters(4.0));
  MechanismRoot2d root = // CAD distance from origin to center of carriage at full retraction
      mech2d.getRoot("Elevator", (3.0 / 2.0) + Units.inchesToMeters(9.053), Units.inchesToMeters(12.689));
  MechanismLigament2d carriage = new MechanismLigament2d("Carriage", 0, 80);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;

    root.append(carriage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    carriage.setLength(inputs.elevatorPositionMeters);
    Logger.recordOutput("Elevator/Mechanism2d", mech2d);
  }

  public Command setExtension(DoubleSupplier meters) {
    return this.run(() -> {
      io.setTarget(meters.getAsDouble());
      Logger.recordOutput("Elevator/Setpoint", meters.getAsDouble());
    });
  }
}
