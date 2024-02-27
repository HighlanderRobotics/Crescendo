// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMetersPerSec = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double[] elevatorStatorCurrentAmps = new double[] {};
    public double[] elevatorSupplyCurrentAmps = new double[] {};
    public double[] elevatorTempCelsius = new double[] {};
  }

  public void updateInputs(final ElevatorIOInputsAutoLogged inputs);

  public void setTarget(final double meters);

  public void setVoltage(final double voltage);

  public default void stop() {
    setVoltage(0);
  }

  public void resetEncoder(final double position);

  public default void resetEncoder() {
    resetEncoder(0.0);
  }
}
