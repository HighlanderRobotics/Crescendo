// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeVelocityRotationsPerSecond = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeStatorCurrentAmps = 0.0;
    public double intakeSupplyCurrentAmps = 0.0;
    public double intakeTemperatureCelsius = 0.0;

    public double centeringVelocityRotationsPerSecond = 0.0;
    public double centeringAppliedVolts = 0.0;
    public double centeringStatorCurrentAmps = 0.0;
    public double centeringSupplyCurrentAmps = 0.0;
    public double centeringTemperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(final IntakeIOInputs inputs);

  /** Run the intake at a specified voltage */
  public void setIntakeVoltage(final double volts);

  /** Run the centering wheels at a specified voltage */
  public void setCenteringVoltage(final double volts);
}
