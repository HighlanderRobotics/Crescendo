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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
  final TalonFX motor = new TalonFX(19, "canivore");

  final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Double> velocity = motor.getVelocity();
  private final StatusSignal<Double> voltage = motor.getMotorVoltage();
  private final StatusSignal<Double> amperage = motor.getStatorCurrent();
  private final StatusSignal<Double> temp = motor.getDeviceTemp();
  private final StatusSignal<Double> position = motor.getPosition();

  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);

  public ClimberIOReal() {
    var config = new TalonFXConfiguration();

    // TODO find all of this
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ClimberSubsystem.SENSOR_TO_MECHANISM_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 20.0;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kG = 0.0;
    config.Slot0.kV = 8.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kS = 0.18;
    config.Slot0.kP = 20.0;
    config.Slot0.kD = 0.0;
    config.MotionMagic.MotionMagicAcceleration = 10.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 4.0;

    motor.getConfigurator().apply(config);
    motor.setPosition(ClimberSubsystem.CLIMBER_MIN_ROTATIONS); // Assume we boot at hard stop

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
    inputs.climberRotations = position.getValueAsDouble();
  }

  @Override
  public void setClimberVoltage(double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setSetpoint(double rotations) {
    motor.setControl(motionMagic.withPosition(rotations));
  }

  @Override
  public void resetPosition(double rotation) {
    motor.setPosition(rotation);
  }
}
