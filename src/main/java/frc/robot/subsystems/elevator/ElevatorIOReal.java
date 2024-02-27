// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

/** Elevator IO using TalonFXs. */
public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX motor = new TalonFX(16);
  private final TalonFX follower = new TalonFX(17);

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage positionVoltage =
      new MotionMagicVoltage(0.0).withEnableFOC(true);

  private final StatusSignal<Double> position = motor.getPosition();
  private final StatusSignal<Double> velocity = motor.getVelocity();
  private final StatusSignal<Double> voltage = motor.getMotorVoltage();
  private final StatusSignal<Double> statorCurrent = motor.getStatorCurrent();
  private final StatusSignal<Double> supplyCurrent = motor.getSupplyCurrent();
  private final StatusSignal<Double> temp = motor.getDeviceTemp();

  private final StatusSignal<Double> followerStatorCurrent = follower.getStatorCurrent();
  private final StatusSignal<Double> followerSupplyCurrent = follower.getSupplyCurrent();
  private final StatusSignal<Double> followerTemp = follower.getDeviceTemp();

  public ElevatorIOReal() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kG = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicAcceleration = 10.0;
    // Estimated from slightly less than motor free speed
    config.MotionMagic.MotionMagicCruiseVelocity =
        50.0 / (ElevatorSubsystem.GEAR_RATIO * 2 * Math.PI * ElevatorSubsystem.DRUM_RADIUS_METERS);

    // Carriage position meters in direction of elevator
    config.Feedback.SensorToMechanismRatio =
        ElevatorSubsystem.GEAR_RATIO * 2 * Math.PI * ElevatorSubsystem.DRUM_RADIUS_METERS;

    motor.getConfigurator().apply(config);
    motor.setPosition(0.0); // Assume we boot 0ed
    follower.getConfigurator().apply(new TalonFXConfiguration());
    follower.setControl(new Follower(motor.getDeviceID(), true));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, voltage, statorCurrent, supplyCurrent, temp, followerStatorCurrent, followerSupplyCurrent, followerTemp);
    motor.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(final ElevatorIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(position, velocity, voltage, statorCurrent, supplyCurrent, temp, followerStatorCurrent, followerSupplyCurrent, followerTemp);
    inputs.elevatorPositionMeters = position.getValueAsDouble();
    inputs.elevatorVelocityMetersPerSec = velocity.getValueAsDouble();
    inputs.elevatorAppliedVolts = voltage.getValueAsDouble();
    inputs.elevatorStatorCurrentAmps = new double[] {statorCurrent.getValueAsDouble(), followerStatorCurrent.getValueAsDouble()};
    inputs.elevatorSupplyCurrentAmps = new double[] {supplyCurrent.getValueAsDouble(), followerSupplyCurrent.getValueAsDouble()};
    inputs.elevatorTempCelsius = new double[] {temp.getValueAsDouble(), followerTemp.getValueAsDouble()};
  }

  @Override
  public void setTarget(final double meters) {
    motor.setControl(positionVoltage.withPosition(meters));
  }

  @Override
  public void setVoltage(final double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void resetEncoder(final double position) {
    motor.setPosition(position);
  }
}
