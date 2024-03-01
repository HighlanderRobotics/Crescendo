// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
  final TalonFX motor = new TalonFX(0, "canivore"); // TODO get id

  final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  final StatusSignal<Double> velocity = motor.getVelocity();
  final StatusSignal<Double> voltage = motor.getMotorVoltage();
  final StatusSignal<Double> amperage = motor.getStatorCurrent();
  final StatusSignal<Double> temp = motor.getDeviceTemp();
  final StatusSignal<Double> position = motor.getPosition();

  private final MotionMagicVoltage motionMagic =
      new MotionMagicVoltage(0.0).withEnableFOC(true);

  public ClimberIOReal() {
    var config = new TalonFXConfiguration();
    config.MotionMagic.MotionMagicAcceleration = 1.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 1.0;

    //TODO find PID values
    motor.getConfigurator().apply(config);
    motor.setPosition(ClimberSubsystem.CLIMBER_MIN_ANGLE.getRotations()); // Assume we boot at hard stop
    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, velocity, voltage, amperage, temp, position);
    motor.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, voltage, amperage, temp, position);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(final ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, voltage, amperage, temp, position);
    inputs.climberVelocityRotationsPerSec = velocity.getValueAsDouble();
    inputs.climberAppliedVolts = voltage.getValueAsDouble();
    inputs.climberCurrentAmps = amperage.getValueAsDouble();
    inputs.climberTempC = temp.getValueAsDouble();
    inputs.climberRotation = Rotation2d.fromRotations(position.getValue());
  }

  @Override
  public void setClimberVoltage(double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }
  @Override
  public void setSetpoint(Rotation2d rotation) {
    motor.setControl(motionMagic.withPosition(rotation.getRotations()));
  }
  @Override
  public void resetPosition(Rotation2d rotation) {
    motor.setPosition(rotation.getRotations());
  }
}
