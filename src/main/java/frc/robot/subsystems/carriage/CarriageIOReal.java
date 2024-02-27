// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.carriage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;

/** Create a CarriageIO that uses a real TalonFX. */
public class CarriageIOReal implements CarriageIO {
  final TalonFX motor = new TalonFX(18);

  final DigitalInput beambreak = new DigitalInput(2);

  final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  final StatusSignal<Double> velocity = motor.getVelocity();
  final StatusSignal<Double> voltage = motor.getMotorVoltage();
  final StatusSignal<Double> statorCurrent = motor.getStatorCurrent();
  final StatusSignal<Double> supplyCurrent = motor.getSupplyCurrent();
  final StatusSignal<Double> temp = motor.getDeviceTemp();

  public CarriageIOReal() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, voltage, statorCurrent, supplyCurrent, temp);
    motor.optimizeBusUtilization();
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(final CarriageIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(velocity, voltage, statorCurrent, supplyCurrent, temp);
    inputs.velocityRotationsPerSecond = velocity.getValueAsDouble();
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.temperatureCelsius = temp.getValueAsDouble();

    inputs.beambreak = beambreak.get();
  }

  /** Run the intake at a specified voltage */
  @Override
  public void setVoltage(final double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }
}
