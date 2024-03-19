// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.carriage;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.utils.components.InvertedDigitalInput;
import frc.robot.utils.logging.TalonFXLogger;

/** Create a CarriageIO that uses a real TalonFX. */
public class CarriageIOReal implements CarriageIO {
  private final TalonFX motor = new TalonFX(18, "canivore");

  private final InvertedDigitalInput beambreak = new InvertedDigitalInput(2);

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  private final TalonFXLogger logger = new TalonFXLogger(motor);

  public CarriageIOReal() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(config);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(final CarriageIOInputsAutoLogged inputs) {
    inputs.carriage = logger.update();

    inputs.beambreak = beambreak.get();
  }

  /** Run the intake at a specified voltage */
  @Override
  public void setVoltage(final double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }
}
