// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  public class ShooterIOInputs {

    // Motor values
    public double velocityRPM;
    public double currentDrawAmps;
    public double temperatureCelsius;
    public double motorOutputVolts;  }

  public abstract void setVelocity(double velocity);

  public abstract void reset(double velocity);

  public abstract ShooterIOInputsAutoLogged updateInputs();
}
