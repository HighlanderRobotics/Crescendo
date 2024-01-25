// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.routing;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.routing.RoutingIOInputsAutoLogged;

/** Add your docs here. */
public interface RoutingIO {
  @AutoLog
  public class RoutingIOInputs {
    // Motor values
    public double velocityRPS;
    public double currentDrawAmps;
    public double temperatureCelsius;
    public double motorOutputVolts;
  }
  
  public abstract RoutingIOInputsAutoLogged updateInputs();

  public abstract void setVelocity(double rps);
}
