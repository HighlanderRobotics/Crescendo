// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.google.common.collect.ImmutableSet;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Registration;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Swerve/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOReal implements ModuleIO {
  // Constants
  private static final boolean IS_TURN_MOTOR_INVERTED = true;

  private final String name;

  // Hardware
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Signals
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  private double lastUpdate = 0;

  // Control modes
  private final VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut turnVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage drivePIDF = new VelocityVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVoltage turnPID = new MotionMagicVoltage(0.0).withEnableFOC(true);

  public ModuleIOReal(ModuleConstants constants) {
    name = constants.prefix();

    driveTalon = new TalonFX(constants.driveID(), "canivore");
    turnTalon = new TalonFX(constants.turnID(), "canivore");
    cancoder = new CANcoder(constants.cancoderID(), "canivore");

    var driveConfig = new TalonFXConfiguration();
    // Current limits
    // TODO: Do we want to limit supply current?
    driveConfig.CurrentLimits.StatorCurrentLimit = Module.DRIVE_STATOR_CURRENT_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyTimeThreshold = Module.DRIVE_SUPPLY_TIME_CUTOFF;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentThreshold = Module.DRIVE_SUPPLY_TIME_CURRENT_LIMIT;
    // Inverts
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Sensor
    // Meters per second
    driveConfig.Feedback.SensorToMechanismRatio =
        (Module.DRIVE_GEAR_RATIO) * (1.0 / (Module.WHEEL_RADIUS * 2 * Math.PI));
    // Controls Gains
    driveConfig.Slot0.kV = 2.0733;
    driveConfig.Slot0.kA = 0.4;
    driveConfig.Slot0.kS = 0.04;
    driveConfig.Slot0.kP = 1.9855;
    driveConfig.Slot0.kD = 0.0;

    driveConfig.MotionMagic.MotionMagicCruiseVelocity = SwerveSubsystem.MAX_LINEAR_SPEED;
    driveConfig.MotionMagic.MotionMagicAcceleration = SwerveSubsystem.MAX_LINEAR_SPEED / 0.75;

    driveTalon.getConfigurator().apply(driveConfig);

    var turnConfig = new TalonFXConfiguration();
    // Current limits
    turnConfig.CurrentLimits.StatorCurrentLimit = Module.TURN_STATOR_CURRENT_LIMIT;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Inverts
    turnConfig.MotorOutput.Inverted =
        IS_TURN_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Fused Cancoder
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.cancoderID();
    turnConfig.Feedback.RotorToSensorRatio = Module.TURN_GEAR_RATIO;
    turnConfig.Feedback.SensorToMechanismRatio = 1.0;
    turnConfig.Feedback.FeedbackRotorOffset =
        0.0; // Is this correct? Cancoder config should handle it
    // Controls Gains
    turnConfig.Slot0.kV = 2.7935;
    turnConfig.Slot0.kA = 0.031543;
    turnConfig.Slot0.kS = 0.28;
    turnConfig.Slot0.kP = 28.579;
    turnConfig.Slot0.kD = 0.68275;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 5500 / Module.TURN_GEAR_RATIO;
    turnConfig.MotionMagic.MotionMagicAcceleration = (5500 * 0.1) / Module.TURN_GEAR_RATIO;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    turnTalon.getConfigurator().apply(turnConfig);

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = constants.cancoderOffset().getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        IS_TURN_MOTOR_INVERTED
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    PhoenixOdometryThread.getInstance()
        .registerSignals(
            new Registration(driveTalon, ImmutableSet.of(drivePosition)),
            new Registration(turnTalon, ImmutableSet.of(turnPosition)));

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY_HZ, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionMeters = drivePosition.getValueAsDouble();
    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    var samples =
        PhoenixOdometryThread.getInstance()
            .samplesSince(lastUpdate, ImmutableSet.of(drivePosition, turnPosition));
    if (!samples.isEmpty()) {
      lastUpdate = samples.get(samples.size() - 1).timestamp();
    }

    inputs.odometryTimestamps = samples.stream().mapToDouble(s -> s.timestamp()).toArray();
    inputs.odometryDrivePositionsMeters =
        samples.stream().filter(s -> s != null).mapToDouble(s -> s.values().get(drivePosition)).toArray();
    inputs.odometryTurnPositions =
        samples.stream()
            // should be after offset + gear ratio
            .map(s -> s.values().get(turnPosition))
            .filter(s -> s != null)
            .map(Rotation2d::fromRotations)
            .toArray(Rotation2d[]::new);
  }

  @Override
  public void setDriveVoltage(final double volts) {
    driveTalon.setControl(driveVoltage.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(final double volts) {
    turnTalon.setControl(turnVoltage.withOutput(volts));
  }

  @Override
  public void setDriveSetpoint(final double metersPerSecond) {
    driveTalon.setControl(drivePIDF.withVelocity(metersPerSecond));
  }

  @Override
  public void setTurnSetpoint(final Rotation2d rotation) {
    turnTalon.setControl(turnPID.withPosition(rotation.getRotations()));
  }

  @Override
  public String getModuleName() {
    return name;
  }
}
