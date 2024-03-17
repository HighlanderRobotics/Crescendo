// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.logging.TalonFXLogger.TalonFXLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public TalonFXLog intake = new TalonFXLog(0, 0, 0, 0, 0, 0);

    public TalonFXLog centering = new TalonFXLog(0, 0, 0, 0, 0, 0);

  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(final IntakeIOInputs inputs);

  /** Run the intake at a specified voltage */
  public void setIntakeVoltage(final double volts);

  public void setIntakeSpeed(final double rps);

  /** Run the centering wheels at a specified voltage */
  public void setCenteringVoltage(final double volts);

  public void setCenteringSpeed(final double rps);
}
