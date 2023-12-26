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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class Module {
  // Constants
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
  public static final double ODOMETRY_FREQUENCY_HZ = 250.0;
  
  // Gear ratios for SDS MK4i L2, adjust as necessary
  public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
  
  public static final double DRIVE_STATOR_CURRENT_LIMIT = 80.0; // TODO bump as needed
  public static final double TURN_STATOR_CURRENT_LIMIT = 40.0;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final String suffix;

  private double lastPositionMeters = 0.0; // Used for delta calculation
  private SwerveModulePosition[] positionDeltas = new SwerveModulePosition[] {};

  public Module(ModuleIO io, String suffix) {
    this.io = io;
    this.suffix = suffix;
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Swerve/Module" + suffix, inputs);

    // Calculate position deltas for odometry
    int deltaCount =
        Math.min(inputs.odometryDrivePositionsRad.length, inputs.odometryTurnPositions.length);
    positionDeltas = new SwerveModulePosition[deltaCount];
    for (int i = 0; i < deltaCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS;
      Rotation2d angle =
          inputs.odometryTurnPositions[i];
      positionDeltas[i] = new SwerveModulePosition(positionMeters - lastPositionMeters, angle);
      lastPositionMeters = positionMeters;
    }
  }

  /** Runs the module closed loop with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    io.setTurnSetpoint(optimizedState.angle);
    io.setDriveSetpoint(optimizedState.speedMetersPerSecond);

    Logger.recordOutput("Swerve/Module" + suffix + " Angle Setpoint", optimizedState.angle);
    Logger.recordOutput("Swerve/Module" + suffix + " Velocity", optimizedState.speedMetersPerSecond);
    Logger.recordOutput("Swerve/Module" + suffix + " Voltage", 0.0); // Closed loop, so we dont directly set voltage

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    io.setTurnSetpoint(new Rotation2d());

    // Open loop drive control
    io.setDriveVoltage(volts);

    Logger.recordOutput("Swerve/Module" + suffix + " Angle Setpoint", 0.0);
    Logger.recordOutput("Swerve/Module" + suffix + " Velocity", 0.0); // Open loop, so we dont directly set velocity
    Logger.recordOutput("Swerve/Module" + suffix + " Voltage", volts);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);

    Logger.recordOutput("Swerve/Module" + suffix + " Angle Setpoint", 0.0);
    Logger.recordOutput("Swerve/Module" + suffix + " Velocity", 0.0);
    Logger.recordOutput("Swerve/Module" + suffix + " Voltage", 0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * WHEEL_RADIUS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position deltas received this cycle. */
  public SwerveModulePosition[] getPositionDeltas() {
    return positionDeltas;
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
