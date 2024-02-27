// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.carriage;

import org.littletonrobotics.junction.AutoLog;

public interface CarriageIO {
  @AutoLog
  public static class CarriageIOInputs {
    public double velocityRotationsPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public boolean beambreak = false;
  }

  public void updateInputs(final CarriageIOInputsAutoLogged inputs);

  public void setVoltage(final double volts);
}
