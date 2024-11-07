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
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Module {
  // Represents per-module constants
  public record ModuleConstants(
      int id, String prefix, int driveID, int turnID, int cancoderID, Rotation2d cancoderOffset) {}

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

  private SwerveModuleState lastSetpoint = new SwerveModuleState();
  private double lastTime = Timer.getFPGATimestamp();

  public Module(final ModuleIO io) {
    this.io = io;
  }

  /** Update inputs without running the rest of the periodic logic. */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs(String.format("Swerve/%s Module", inputs.prefix), inputs);
    Logger.recordOutput(
        String.format("Swerve/%s Module/Voltage Available", inputs.prefix),
        Math.abs(inputs.driveAppliedVolts - RoboRioDataJNI.getVInVoltage()));
  }

  /** Runs the module closed loop with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    return runSetpoint(state, 0.0, 0.0);
  }

  /** Runs the module closed loop with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(
      SwerveModuleState state, double forceXNewtons, double forceYNewtons) {
    // Optimize state based on current angle
    final var optimizedState = SwerveModuleState.optimize(state, getAngle());
    // force on the motor is the total force vector projected onto the velocity vector
    // to project a onto b take ||a||*cos(theta) where theta is the angle between the two vectors
    // We want the magnitude of the projection, so we can ignore the direction of this later
    final var theta = Math.atan2(forceYNewtons, forceXNewtons) - inputs.turnPosition.getRadians();
    final double forceNewtons = Math.hypot(forceXNewtons, forceYNewtons) * Math.cos(theta);

    io.setTurnSetpoint(optimizedState.angle);
    io.setDriveSetpoint(
        optimizedState.speedMetersPerSecond
            * Math.cos(optimizedState.angle.minus(inputs.turnPosition).getRadians()),
        forceNewtons);
    Logger.recordOutput(
        String.format("Swerve/%s Module/Force Feedforward", inputs.prefix), forceNewtons);
    lastSetpoint = optimizedState;
    lastTime = Timer.getFPGATimestamp();
    return optimizedState;
  }

  /**
   * Runs the module open loop with the specified setpoint state, velocity in volts. Returns the
   * optimized state.
   */
  public SwerveModuleState runVoltageSetpoint(SwerveModuleState state, boolean focEnabled) {
    // Optimize state based on current angle
    final var optimizedState = SwerveModuleState.optimize(state, getAngle());
    Logger.recordOutput(
        String.format("Swerve/%s Module/Voltage Target", inputs.prefix),
        optimizedState.speedMetersPerSecond);

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

  /** Returns this modules prefix ie "Back Left" */
  public String getPrefix() {
    return inputs.prefix;
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
}
