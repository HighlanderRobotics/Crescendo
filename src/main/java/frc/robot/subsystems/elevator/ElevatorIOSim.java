// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
  ElevatorSim physicsSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          ElevatorSubsystem.GEAR_RATIO,
          Units.lbsToKilograms(20.0),
          ElevatorSubsystem.DRUM_RADIUS_METERS,
          0.0,
          1.0,
          true,
          1.0);
  double volts = 0.0;
  ProfiledPIDController pid = new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(1.0, 1.0));
  ElevatorFeedforward ff = new ElevatorFeedforward(0.0, 0.48, 2.5);

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }
    physicsSim.update(0.020);
    inputs.elevatorPositionMeters = physicsSim.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = physicsSim.getVelocityMetersPerSecond();
    inputs.elevatorAppliedVolts = volts;
    inputs.elevatorCurrentAmps = new double[] {physicsSim.getCurrentDrawAmps()};
    inputs.elevatorTempCelsius = new double[] {20.0};
  }

  @Override
  public void setTarget(double meters) {
    setVoltage(
        pid.calculate(physicsSim.getPositionMeters(), meters)
            + ff.calculate(pid.getSetpoint().velocity));
  }

  @Override
  public void setVoltage(double voltage) {
    volts = voltage;
    physicsSim.setInput(voltage);
  }
}
