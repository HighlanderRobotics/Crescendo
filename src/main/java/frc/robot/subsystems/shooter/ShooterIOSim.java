// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
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

  DCMotorSim leftFlywheelSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), ShooterSubystem.FLYWHEEL_RATIO, 0.001);
  DCMotorSim rightFlywheelSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), ShooterSubystem.FLYWHEEL_RATIO, 0.001);

  ProfiledPIDController pivotController =
      new ProfiledPIDController(1.0, 0.0, 1.0, new Constraints(10.0, 10.0));
  ArmFeedforward pivotFF = new ArmFeedforward(0.0, 0.12, 0.8);

  private final PIDController leftFlywheelController = new PIDController(0.5, 0.0, 0.0);
  private final PIDController rightFlywheelController = new PIDController(0.5, 0.0, 0.0);
  SimpleMotorFeedforward flywheelFF = new SimpleMotorFeedforward(0.0, 0.0925);

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    leftFlywheelSim.update(0.020);
    rightFlywheelSim.update(0.020);
    pivotSim.update(0.020);

    inputs.pivotRotation = Rotation2d.fromRadians(pivotSim.getAngleRads());
    inputs.pivotVelocityRotationsPerSecond =
        Units.rotationsToRadians(pivotSim.getVelocityRadPerSec());
    inputs.pivotVoltage = 0.0;
    inputs.pivotStatorCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.pivotTempC = 0.0;

    inputs.flywheelLeftVelocityRotationsPerSecond = leftFlywheelSim.getAngularVelocityRPM() / 60.0;
    inputs.flywheelLeftVoltage = 0.0;
    inputs.flywheelLeftStatorCurrentAmps = leftFlywheelSim.getCurrentDrawAmps();
    inputs.flywheelLeftTempC = 0.0;

    inputs.flywheelRightVelocityRotationsPerSecond =
        rightFlywheelSim.getAngularVelocityRPM() / 60.0;
    inputs.flywheelRightVoltage = 0.0;
    inputs.flywheelRightStatorCurrentAmps = rightFlywheelSim.getCurrentDrawAmps();
    inputs.flywheelRightTempC = 0.0;
  }

  public void setPivotVoltage(final double voltage) {
    pivotSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  public void setPivotSetpoint(final Rotation2d rotation) {
    setPivotVoltage(
        pivotController.calculate(pivotSim.getAngleRads(), rotation.getRadians())
            + pivotFF.calculate(
                pivotController.getSetpoint().position, pivotController.getSetpoint().velocity));
  }

  public void setFlywheelVelocity(final double left, final double right) {
    setFlywheelVoltage(
        leftFlywheelController.calculate(
                leftFlywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2), left)
            + flywheelFF.calculate(left),
        rightFlywheelController.calculate(
                rightFlywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2), right)
            + flywheelFF.calculate(right));
  }

  public void setFlywheelVoltage(final double left, final double right) {
    leftFlywheelSim.setInputVoltage(MathUtil.clamp(left, -12, 12));
    rightFlywheelSim.setInputVoltage(MathUtil.clamp(right, -12, 12));
  }

  @Override
  public void resetPivotPostion(Rotation2d rotation) {
    pivotSim.setState(rotation.getRadians(), 0.0);
  }
}
