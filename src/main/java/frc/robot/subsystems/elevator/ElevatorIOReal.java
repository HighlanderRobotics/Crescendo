// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO {
  TalonFX motor = new TalonFX(10);

  VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true);

  StatusSignal<Double> position = motor.getPosition();
  StatusSignal<Double> velocity = motor.getVelocity();
  StatusSignal<Double> voltage = motor.getMotorVoltage();
  StatusSignal<Double> current = motor.getStatorCurrent();
  StatusSignal<Double> temp = motor.getDeviceTemp();

  public ElevatorIOReal() {
    var config = new TalonFXConfiguration();

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

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, voltage, current, temp);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    inputs.elevatorPositionMeters = position.getValueAsDouble();
    inputs.elevatorVelocityMetersPerSec = velocity.getValueAsDouble();
    inputs.elevatorAppliedVolts = voltage.getValueAsDouble();
    inputs.elevatorCurrentAmps = new double[] {current.getValueAsDouble()};
    inputs.elevatorTempCelsius = new double[] {temp.getValueAsDouble()};
  }

  public void setTarget(double meters) {
    motor.setControl(positionVoltage.withPosition(meters));
  }

  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  public void resetEncoder(double position) {
    motor.setPosition(position);
  }
}
