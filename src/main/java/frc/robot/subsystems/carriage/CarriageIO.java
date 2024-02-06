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
    public double[] currentAmps = new double[] {};
    public double[] temperatureCelsius = new double[] {};
    // Not in cad as of 1/28 but requested
    public boolean beambreak = false;
  }

  public default void updateInputs(CarriageIOInputsAutoLogged inputs) {}

  public default void setVoltage(double volts) {}
}
