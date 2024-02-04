// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
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

  public abstract ShooterIOInputsAutoLogged updateInputs();

  public default void setFlywheelVoltage(final double left, final double right) {} // set voltage

  public default void setFlywheelVelocity(final double left, final double right) {} // set speed

  public default void setPivotVoltage(final double voltage) {} // set speed

  public default void setPivotSetpoint(final Rotation2d rotation) {} // set specfic rotation

  public default void resetPivotPostion(final Rotation2d rotation) {} // reset specfic rotation
}
