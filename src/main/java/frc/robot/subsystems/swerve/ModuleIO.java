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
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Samples;
import frc.robot.utils.NullableRotation2d;
import frc.robot.utils.NullableDouble;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};

    public NullableDouble[] odometryDrivePositionsMeters = new NullableDouble[] {};
    public NullableRotation2d[] odometryTurnPositions = new NullableRotation2d[] {};
    public double[] odometryTimestamps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(final ModuleIOInputs inputs, final List<Samples> asyncOdometrySamples);

  /** Run the drive motor at the specified voltage. */
  public void setDriveVoltage(final double volts);

  /** Use onboard PIDF to run the drive motor at the specified speed */
  public default void setDriveSetpoint(final double metersPerSecond) {
    setDriveSetpoint(metersPerSecond, 0.0);
  }

  /** Use onboard PIDF to run the drive motor at the specified speed */
  public void setDriveSetpoint(final double metersPerSecond, final double metersPerSecondSquared);

  /** Run the turn motor at the specified voltage. */
  public void setTurnVoltage(final double volts);

  /** Use onboard PIDF to run the turn motor to the specified rotation */
  public void setTurnSetpoint(final Rotation2d rotation);

  /** Gets the name of the swerve module for logging purposes, should be constant per-module. */
  public String getModuleName();
}
