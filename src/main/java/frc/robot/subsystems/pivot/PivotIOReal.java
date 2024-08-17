// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOReal implements PivotIO {
  private final TalonFX pivotMotor = new TalonFX(10, "canivore");

  private final StatusSignal<Double> pivotVelocity = pivotMotor.getVelocity();
  private final StatusSignal<Double> pivotVoltage = pivotMotor.getMotorVoltage();
  private final StatusSignal<Double> pivotAmps = pivotMotor.getStatorCurrent();
  private final StatusSignal<Double> pivotTempC = pivotMotor.getDeviceTemp();
  private final StatusSignal<Double> pivotRotations = pivotMotor.getPosition();

  private final VoltageOut pivotVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage pivotMotionMagic =
      new MotionMagicVoltage(0.0).withEnableFOC(true);

  public PivotIOReal() {
    var pivotConfig = new TalonFXConfiguration();

    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotConfig.Feedback.SensorToMechanismRatio = PivotSubsystem.RATIO;

    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40.0;

    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.Slot0.kG = 0.5;
    pivotConfig.Slot0.kV = 7.2;
    pivotConfig.Slot0.kA = 0.1;
    pivotConfig.Slot0.kS = 0.0;
    pivotConfig.Slot0.kP = 400.0;
    pivotConfig.Slot0.kD = 0.0;

    pivotConfig.MotionMagic.MotionMagicAcceleration = 1.0;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;

    pivotMotor.getConfigurator().apply(pivotConfig);
    pivotMotor.setPosition(PivotSubsystem.MIN_ANGLE.getRotations()); // Assume we boot at hard stop
    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, pivotVelocity, pivotVoltage, pivotAmps, pivotTempC, pivotRotations);
    pivotMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(pivotRotations, pivotVelocity, pivotVoltage, pivotAmps, pivotTempC);

    inputs.pivotRotation = Rotation2d.fromRotations(pivotRotations.getValue());
    inputs.pivotVelocityRotationsPerSecond = pivotVelocity.getValue();
    inputs.pivotVoltage = pivotVoltage.getValue();
    inputs.pivotAmps = pivotAmps.getValue();
    inputs.pivotTempC = pivotTempC.getValue();
  }

  public void resetPivotPosition(final Rotation2d rotation) {
    pivotMotor.setPosition(rotation.getRotations());
  }

  public void setPivotVoltage(final double voltage) {
    pivotMotor.setControl(pivotVoltageOut.withOutput(voltage));
  }

  public void setPivotSetpoint(final Rotation2d rotation) {
    pivotMotor.setControl(pivotMotionMagic.withPosition(rotation.getRotations()));
  }
}
