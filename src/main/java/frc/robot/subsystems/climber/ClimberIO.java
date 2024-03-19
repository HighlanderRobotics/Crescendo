// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    // Not refactored bc removed
    public double climberVelocityRotationsPerSec = 0.0;
    public double climberAppliedVolts = 0.0;
    public double climberCurrentAmps = 0.0;
    public double climberTempC = 0.0;
    public double climberRotations = 0.0; // Total rotations of the spool
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(final ClimberIOInputs inputs);

  /** Run the climber at a specified voltage */
  public void setClimberVoltage(final double volts);

  public void setSetpoint(final double rotations);

  public void resetPosition(final double rotations);
}
