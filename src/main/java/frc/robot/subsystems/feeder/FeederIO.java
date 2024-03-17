// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.logging.TalonFXLogger.TalonFXLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public TalonFXLog feeder = new TalonFXLog(0, 0, 0, 0, 0, 0);

    public boolean firstBeambreak = false;
    public boolean lastBeambreak = false;
  }

  public void updateInputs(final FeederIOInputsAutoLogged inputs);

  public void setVoltage(final double volts);

  public void setVelocity(final double velocity);
}
