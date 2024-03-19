// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.carriage;

import frc.robot.utils.logging.TalonFXLogger.TalonFXLog;
import org.littletonrobotics.junction.AutoLog;

public interface CarriageIO {
  @AutoLog
  public static class CarriageIOInputs {
    public TalonFXLog carriage = new TalonFXLog(0, 0, 0, 0, 0, 0);
    public boolean beambreak = false;
  }

  public void updateInputs(final CarriageIOInputsAutoLogged inputs);

  public void setVoltage(final double volts);
}
