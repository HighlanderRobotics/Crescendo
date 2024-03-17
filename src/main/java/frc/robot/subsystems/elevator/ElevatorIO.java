// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.logging.TalonFXLogger.TalonFXLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public TalonFXLog leader;
    public TalonFXLog follower;
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
