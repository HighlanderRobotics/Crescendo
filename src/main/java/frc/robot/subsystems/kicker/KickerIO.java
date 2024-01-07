// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface KickerIO {
  @AutoLog
  public class KickerIOInputs {

    // Motor values
    public double velocityRPM;
    public double currentDrawAmps;
    public double temperatureCelsius;
    public double motorOutputVolts;
    public double positionRotations;
  }

  public abstract void setPosition(double degrees);

  public abstract void reset(double degrees);

  public abstract KickerIOInputsAutoLogged updateInputs();
}
