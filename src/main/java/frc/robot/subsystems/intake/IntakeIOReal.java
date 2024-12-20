// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

/** Intake IO implementation for TalonFX motors. */
public class IntakeIOReal implements IntakeIO {
  private final TalonFX intakeMotor = new TalonFX(14, "canivore");
  private final TalonFX centeringMotor = new TalonFX(15, "canivore");

  private final VoltageOut intakeVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage intakeVelocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut centeringVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage centeringVelocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true);

  private final StatusSignal<Double> intakeVelocity = intakeMotor.getVelocity();
  private final StatusSignal<Double> intakeVoltage = intakeMotor.getMotorVoltage();
  private final StatusSignal<Double> intakeAmperage = intakeMotor.getStatorCurrent();
  private final StatusSignal<Double> intakeTemp = intakeMotor.getDeviceTemp();

  private final StatusSignal<Double> centeringVelocity = centeringMotor.getVelocity();
  private final StatusSignal<Double> centeringVoltage = centeringMotor.getMotorVoltage();
  private final StatusSignal<Double> centeringAmperage = centeringMotor.getStatorCurrent();
  private final StatusSignal<Double> centeringTemp = centeringMotor.getDeviceTemp();

  public IntakeIOReal() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 15.0;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakeConfig.Slot0.kV = (12.0 * 60.0) / 5800;
    intakeConfig.Slot0.kP = 1.0;
    intakeConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

    intakeMotor.getConfigurator().apply(intakeConfig);

    var centeringConfig = new TalonFXConfiguration();
    centeringConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    centeringConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    centeringConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    centeringConfig.Slot0.kV = (12.0 * 60.0) / 5800;
    centeringConfig.Slot0.kP = 0.1;
    centeringConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

    centeringMotor.getConfigurator().apply(centeringConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeVelocity,
        intakeVoltage,
        intakeAmperage,
        intakeTemp,
        centeringVelocity,
        centeringVoltage,
        centeringAmperage,
        centeringTemp);
    intakeMotor.optimizeBusUtilization();
    centeringMotor.optimizeBusUtilization();
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        intakeVelocity,
        intakeVoltage,
        intakeAmperage,
        intakeTemp,
        centeringVelocity,
        centeringVoltage,
        centeringAmperage,
        centeringTemp);
    inputs.intakeVelocityRotationsPerSecond = intakeVelocity.getValueAsDouble();
    inputs.intakeAppliedVolts = intakeVoltage.getValueAsDouble();
    inputs.intakeCurrentAmps = intakeAmperage.getValueAsDouble();
    inputs.intakeTemperatureCelsius = intakeTemp.getValueAsDouble();

    inputs.centeringVelocityRotationsPerSecond = centeringVelocity.getValueAsDouble();
    inputs.centeringAppliedVolts = centeringVoltage.getValueAsDouble();
    inputs.centeringCurrentAmps = centeringAmperage.getValueAsDouble();
    inputs.centeringTemperatureCelsius = centeringTemp.getValueAsDouble();
  }

  /** Run the intake at a specified voltage */
  @Override
  public void setIntakeVoltage(final double volts) {
    intakeMotor.setControl(intakeVoltageOut.withOutput(volts));
  }

  /** Run the intake at a specified voltage */
  @Override
  public void setCenteringVoltage(final double volts) {
    centeringMotor.setControl(centeringVoltageOut.withOutput(volts));
  }

  @Override
  public void setIntakeSpeed(final double rps) {
    intakeMotor.setControl(intakeVelocityVoltage.withVelocity(rps));
  }

  @Override
  public void setCenteringSpeed(final double rps) {
    centeringMotor.setControl(centeringVelocityVoltage.withVelocity(rps));
  }
}
