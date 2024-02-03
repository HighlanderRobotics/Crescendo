// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.components.ReversibleDigitalInput;

/** Intake IO implementation for TalonFX motors. */
public class IntakeIOReal implements IntakeIO {
  TalonFX motor = new TalonFX(-1);

  VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  StatusSignal<Double> velocity = motor.getVelocity();
  StatusSignal<Double> voltage = motor.getMotorVoltage();
  StatusSignal<Double> amperage = motor.getStatorCurrent();
  StatusSignal<Double> temp = motor.getDeviceTemp();

  public IntakeIOReal() {
    var config = new TalonFXConfiguration();

    motor.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, voltage, amperage, temp);
    motor.optimizeBusUtilization();
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRotationsPerSecond = velocity.getValueAsDouble();
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = new double[] {amperage.getValueAsDouble()};
    inputs.temperatureCelsius = new double[] {temp.getValueAsDouble()};
  }

  /** Run the intake at a specified voltage */
  public void setIntakeVoltage(final double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }
}
