// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
  final TalonFX motor = new TalonFX(0, "canivore"); // TODO get id

  final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  final StatusSignal<Double> velocity = motor.getVelocity();
  final StatusSignal<Double> voltage = motor.getMotorVoltage();
  final StatusSignal<Double> amperage = motor.getStatorCurrent();
  final StatusSignal<Double> temp = motor.getDeviceTemp();

  public ClimberIOReal() {
    var config = new TalonFXConfiguration();
    motor.getConfigurator().apply(config); // TODO add configs

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, voltage, amperage, temp);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(final ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, voltage, amperage, temp);
    inputs.climberVelocityRotationsPerSec = velocity.getValueAsDouble();
    inputs.climberAppliedVolts = voltage.getValueAsDouble();
    inputs.climberCurrentAmps = amperage.getValueAsDouble();
    inputs.climberTempC = temp.getValueAsDouble();
  }

  @Override
  public void setClimberVoltage(double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }
}
