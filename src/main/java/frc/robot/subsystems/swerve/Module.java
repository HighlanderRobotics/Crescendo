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

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Samples;
import frc.robot.utils.NullableDouble;
import frc.robot.utils.NullableRotation2d;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Module {
  // Represents per-module constants
  public record ModuleConstants(
      String prefix, int driveID, int turnID, int cancoderID, Rotation2d cancoderOffset) {}

  // Global constants
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
  public static final double ODOMETRY_FREQUENCY_HZ = 250.0;

  // Gear ratios for SDS MK4i L3.5, adjust as necessary
  // These numbers are taken from SDS's website
  // They are the gear tooth counts for each stage of the modules' gearboxes
  public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double DRIVE_ROTOR_TO_METERS =
      (Module.DRIVE_GEAR_RATIO) * (1.0 / (Module.WHEEL_RADIUS * 2 * Math.PI));
  public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  public static final double TURN_STATOR_CURRENT_LIMIT = 40.0;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  private SwerveModuleState lastSetpoint = new SwerveModuleState();

  public Module(final ModuleIO io) {
    this.io = io;
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs(final List<Samples> asyncOdometrySamples) {
    io.updateInputs(inputs, asyncOdometrySamples);
  }

  public void periodic() {
    Logger.processInputs(String.format("Swerve/%s Module", io.getModuleName()), inputs);
    Logger.recordOutput(
        String.format("Swerve/%s Module/Voltage Available", io.getModuleName()),
        Math.abs(inputs.driveAppliedVolts - RoboRioDataJNI.getVInVoltage()));

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      NullableDouble positionMeters = inputs.odometryDrivePositionsMeters[i];
      NullableRotation2d angle = inputs.odometryTurnPositions[i];
      if (angle.get() == null || positionMeters.get() == null) {
        odometryPositions[i] = null; // SwerveSubsystem deals with this
      } else {
        odometryPositions[i] = new SwerveModulePosition(positionMeters.get(), angle.get());
      }
    }
  }

  /** Runs the module closed loop with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    final var optimizedState = SwerveModuleState.optimize(state, getAngle());

    io.setTurnSetpoint(optimizedState.angle);
    io.setDriveSetpoint(
        optimizedState.speedMetersPerSecond
            * Math.cos(optimizedState.angle.minus(inputs.turnPosition).getRadians()),
        (optimizedState.speedMetersPerSecond - lastSetpoint.speedMetersPerSecond) / 0.020);

    lastSetpoint = optimizedState;
    return optimizedState;
  }

  /**
   * Runs the module open loop with the specified setpoint state, velocity in volts. Returns the
   * optimized state.
   */
  public SwerveModuleState runVoltageSetpoint(SwerveModuleState state, boolean focEnabled) {
    // Optimize state based on current angle
    final var optimizedState = SwerveModuleState.optimize(state, getAngle());

    io.setTurnSetpoint(optimizedState.angle);
    io.setDriveVoltage(
        optimizedState.speedMetersPerSecond
            * Math.cos(optimizedState.angle.minus(inputs.turnPosition).getRadians()),
        focEnabled);

    return optimizedState;
  }

  /**
   * Runs the module open loop with the specified setpoint state, velocity in volts. Returns the
   * optimized state.
   */
  public SwerveModuleState runVoltageSetpoint(SwerveModuleState state) {
    return runVoltageSetpoint(state, true);
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runDriveCharacterization(double volts) {
    // Closed loop turn control
    io.setTurnSetpoint(Rotation2d.fromRotations(0.0));

    // Open loop drive control
    io.setDriveVoltage(volts);
  }

  /** Runs the module angle with the specified voltage while not moving the drive motor */
  public void runSteerCharacterization(double volts) {
    io.setTurnVoltage(volts);
    io.setDriveVoltage(0.0);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /** Returns the current turn angle of the module at normal sampling frequency. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters at normal sampling frequency. */
  public double getPositionMeters() {
    return inputs.drivePositionMeters;
  }

  /**
   * Returns the current drive velocity of the module in meters per second withat normal sampling
   * frequency.
   */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityMetersPerSec;
  }

  /** Returns the module position (turn angle and drive position) at normal sampling frequency. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity) at normal sampling frequency. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in meters/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityMetersPerSec;
  }

  /** Returns the timestamps of the samples received this cycle from PhoenixOdometryThread. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module positions received this cycle from PhoenixOdometryThread. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }
}
