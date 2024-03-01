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
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.google.common.collect.ImmutableSet;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Registration;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(SwerveSubsystem.PIGEON_ID, "canivore");
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();
  private double lastUpdate = 0;

  public GyroIOPigeon2() {
    var config = new Pigeon2Configuration();
    pigeon.getConfigurator().apply(config);
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Module.ODOMETRY_FREQUENCY_HZ);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
    PhoenixOdometryThread.getInstance()
        .registerSignals(new Registration(pigeon, ImmutableSet.of(yaw)));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    var samples =
        PhoenixOdometryThread.getInstance().samplesSince(lastUpdate, ImmutableSet.of(yaw));
    if (!samples.isEmpty()) {
      lastUpdate = samples.get(samples.size() - 1).timestamp();
    }

    inputs.odometryYawPositions =
        samples.stream()
            .map(s -> s.values().get(yaw))
            .filter(s -> s != null)
            .map(Rotation2d::fromDegrees)
            .toArray(Rotation2d[]::new);
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    pigeon.setYaw(yaw.getDegrees());
  }
}
