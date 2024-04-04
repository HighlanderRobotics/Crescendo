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
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import frc.robot.utils.NullableRotation2d;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public NullableRotation2d[] odometryYawPositions = new NullableRotation2d[] {};
    public double[] odometryTimestamps = new double[] {};
    public double yawVelocityRadPerSec = 0.0;
  }

  public void updateInputs(final GyroIOInputs inputs, final List<Samples> asyncOdometrySamples);

  public void setYaw(final Rotation2d yaw);
}
