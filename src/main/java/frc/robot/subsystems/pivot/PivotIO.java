// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public Rotation2d pivotRotation = new Rotation2d();
    public double pivotVelocityRotationsPerSecond = 0.0;
    public double pivotVoltage = 0.0;
    public double pivotAmps = 0.0;
    public double pivotTempC = 0.0;
  }

  public void updateInputs(final PivotIOInputsAutoLogged inputs);

  public void setPivotVoltage(final double voltage);

  public void setPivotSetpoint(final Rotation2d rotation);

  public void resetPivotPosition(final Rotation2d rotation);
}
