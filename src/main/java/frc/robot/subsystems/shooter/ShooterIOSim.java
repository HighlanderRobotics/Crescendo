// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO {
  SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          ShooterSubystem.PIVOT_RATIO,
          0.85,
          Units.feetToMeters(12),
          -1.0,
          2.0,
          true,
          0);

  DCMotorSim flywheelSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(2), ShooterSubystem.FLYWHEEL_RATIO, 0.00203677199);

  ProfiledPIDController pivotController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0));
  ArmFeedforward pivotFF = new ArmFeedforward(0.0, 0.0, 0.0);

  PIDController flywheelController = new PIDController(0.0, 0.0, 0.0);
  SimpleMotorFeedforward flywheelFF = new SimpleMotorFeedforward(0.0, 0.0);

  @Override
  public ShooterIOInputsAutoLogged updateInputs() {
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    inputs.pivotRotation = Rotation2d.fromRadians(pivotSim.getAngleRads());
    inputs.pivotVelocityRotationsPerSecond =
        Units.rotationsToRadians(pivotSim.getVelocityRadPerSec());
    inputs.pivotVoltage = 0.0;
    inputs.pivotAmps = pivotSim.getCurrentDrawAmps();
    inputs.pivotTempC = 0.0;

    inputs.flywheelVelocityRotationsPerSecond = flywheelSim.getAngularVelocityRPM() / 60.0;
    inputs.flywheelVoltage = 0.0;
    inputs.flywheelAmps = new double[] {flywheelSim.getCurrentDrawAmps()};
    inputs.flywheelTempC = new double[] {0.0};

    return inputs;
  }

  public void setPivotVoltage(final double voltage) {
    pivotSim.setInput(voltage);
  }

  public void setPivotSetpoint(final Rotation2d rotation) {
    setPivotVoltage(
        pivotController.calculate(pivotSim.getAngleRads(), rotation.getRadians())
            + pivotFF.calculate(
                pivotController.getSetpoint().position, pivotController.getSetpoint().velocity));
  }

  public void setFlywheelVelocity(final double rps) {
    setFlywheelVoltage(
        flywheelController.calculate(flywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2), rps)
            + flywheelFF.calculate(rps));
  }

  public void setFlywheelVoltage(final double voltage) {
    flywheelSim.setInput(voltage);
  }
}
