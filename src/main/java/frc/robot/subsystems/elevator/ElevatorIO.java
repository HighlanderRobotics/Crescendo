// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMetersPerSec = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double[] elevatorCurrentAmps = new double[] {};
    public double[] elevatorTempCelsius = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputsAutoLogged inputs) {}

  public default void setTarget(double meters) {}

  public default void setVoltage(double voltage) {}

  public default void stop() {
    setVoltage(0);
  }

  public default void resetEncoder(double position) {}

  public default void resetEncoder() {
    resetEncoder(0.0);
  }
}
