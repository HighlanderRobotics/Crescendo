// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public Rotation2d pivotRotation = new Rotation2d();
    public double pivotVelocityRotationsPerSecond = 0.0;
    public double pivotVoltage = 0.0;
    public double pivotAmps = 0.0;
    public double pivotTempC = 0.0;

    public double flywheelLeftVoltage = 0.0;
    public double flywheelLeftVelocityRotationsPerSecond = 0.0;
    public double flywheelLeftAmps = 0.0;
    public double flywheelLeftTempC = 0.0;

    public double flywheelRightVoltage = 0.0;
    public double flywheelRightVelocityRotationsPerSecond = 0.0;
    public double flywheelRightAmps = 0.0;
    public double flywheelRightTempC = 0.0;
  }

  public void updateInputs(final ShooterIOInputsAutoLogged inputs);

  public void setFlywheelVoltage(final double left, final double right);

  public void setFlywheelVelocity(final double left, final double right);

  public void setPivotVoltage(final double voltage);

  public void setPivotSetpoint(final Rotation2d rotation);

  public void resetPivotPosition(final Rotation2d rotation);
}
