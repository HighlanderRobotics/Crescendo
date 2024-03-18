// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.utils.logging.TalonFXLogger.TalonFXLog;

/** WPILib physics model based elevator sim. */
public class ElevatorIOSim implements ElevatorIO {
  ElevatorSim physicsSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          ElevatorSubsystem.GEAR_RATIO,
          // Add half of first stage mass bc its on a 2:1 ratio compared to carriage
          Units.lbsToKilograms(10.8 + (2.5 / 2)),
          ElevatorSubsystem.DRUM_RADIUS_METERS,
          0.0,
          Units.inchesToMeters(32.0),
          true,
          0.0);
  double volts = 0.0;
  ProfiledPIDController pid = new ProfiledPIDController(40.0, 0.0, 0.0, new Constraints(10.0, 5.0));
  ElevatorFeedforward ff = new ElevatorFeedforward(0.0, 0.06, 12.6);

  @Override
  public void updateInputs(final ElevatorIOInputsAutoLogged inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }
    physicsSim.update(0.020);
    inputs.leader =
        new TalonFXLog(
            volts,
            physicsSim.getCurrentDrawAmps(),
            physicsSim.getCurrentDrawAmps(),
            0.0,
            physicsSim.getPositionMeters(),
            physicsSim.getVelocityMetersPerSecond());
  }

  @Override
  public void setTarget(final double meters) {
    setVoltage(
        pid.calculate(physicsSim.getPositionMeters(), meters)
            + ff.calculate(pid.getSetpoint().velocity));
  }

  @Override
  public void setVoltage(final double voltage) {
    volts = voltage;
    physicsSim.setInputVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  @Override
  public void resetEncoder(final double position) {
    physicsSim.setState(position, 0.0);
  }
}
