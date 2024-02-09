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
import edu.wpi.first.wpilibj.DigitalInput;

/** Feeder IO using a TalonFX. */
public class FeederIOReal implements FeederIO {
  TalonFX motor = new TalonFX(22);

  DigitalInput firstBeambreak = new DigitalInput(0);
  DigitalInput lastBeambreak = new DigitalInput(1);

  StatusSignal<Double> velocity = motor.getVelocity();
  StatusSignal<Double> voltage = motor.getMotorVoltage();
  StatusSignal<Double> current = motor.getStatorCurrent();
  StatusSignal<Double> temp = motor.getDeviceTemp();

  VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

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
  public void updateInputs(FeederIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(velocity, voltage, current, temp);

    inputs.feederVelocityRotationsPerSec = velocity.getValue();
    inputs.feederAppliedVolts = voltage.getValue();
    inputs.feederCurrentAmps = current.getValue();
    inputs.feederTempC = temp.getValue();

    inputs.firstBeambreak = firstBeambreak.get();
    inputs.lastBeambreak = lastBeambreak.get();
  }
}
