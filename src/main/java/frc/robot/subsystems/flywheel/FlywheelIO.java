// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double voltage = 0.0;
    public double velocityRotationsPerSecond = 0.0;
    public double amps = 0.0;
    public double tempC = 0.0;
  }

  public void updateInputs(final FlywheelIOInputsAutoLogged inputs);

  public void setVoltage(final double volts);

  public void setVelocity(final double rps);

  public void setCurrentLimit(final double stator, final double supply);
}
