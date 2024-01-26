// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.routing;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface RoutingIO {
  @AutoLog
  public static class RoutingIOInputs {
    // Motor values
    public double velocityRPS = 0.0;
    public double currentDrawAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double motorOutputVolts = 0.0;
  }

  public abstract void updateInputs(RoutingIOInputsAutoLogged inputs);

  public abstract void setVelocity(double rps);

  public abstract void setVoltage(double voltage);
}
