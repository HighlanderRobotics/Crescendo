// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.utils.components.InvertedDigitalInput;
import frc.robot.utils.logging.TalonFXLogger;

/** Feeder IO using a TalonFX. */
public class FeederIOReal implements FeederIO {
  private final TalonFX motor = new TalonFX(13, "canivore");

  private final InvertedDigitalInput firstBeambreak = new InvertedDigitalInput(0);
  private final InvertedDigitalInput lastBeambreak = new InvertedDigitalInput(1);

  private final TalonFXLogger logger = new TalonFXLogger(motor);

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

  public FeederIOReal() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40.0;

    config.Slot0.kV = 0.12;
    config.Slot0.kP = 0.1;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(final FeederIOInputsAutoLogged inputs) {
    inputs.feeder = logger.update();

    inputs.firstBeambreak = firstBeambreak.get();
    inputs.lastBeambreak = lastBeambreak.get();
  }

  @Override
  public void setVoltage(final double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setVelocity(final double velocity) {
    motor.setControl(velocityVoltage.withVelocity(velocity));
  }
}
