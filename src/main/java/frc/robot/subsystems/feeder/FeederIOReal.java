// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.utils.components.ReversibleDigitalInput;

/** Feeder IO using a TalonFX. */
public class FeederIOReal implements FeederIO {
  private final TalonFX motor = new TalonFX(22);

  private final ReversibleDigitalInput firstBeambreak = new ReversibleDigitalInput(0, false);
  private final ReversibleDigitalInput lastBeambreak = new ReversibleDigitalInput(1, false);

  private final StatusSignal<Double> velocity = motor.getVelocity();
  private final StatusSignal<Double> voltage = motor.getMotorVoltage();
  private final StatusSignal<Double> current = motor.getStatorCurrent();
  private final StatusSignal<Double> temp = motor.getDeviceTemp();

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  public FeederIOReal() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 20.0;

    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, voltage, current, temp);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(final FeederIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(velocity, voltage, current, temp);

    inputs.feederVelocityRotationsPerSec = velocity.getValue();
    inputs.feederAppliedVolts = voltage.getValue();
    inputs.feederCurrentAmps = current.getValue();
    inputs.feederTempC = temp.getValue();

    inputs.firstBeambreak = firstBeambreak.get();
    inputs.lastBeambreak = lastBeambreak.get();
  }

  @Override
  public void setVoltage(final double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }
}
