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

    public double flywheelAmps = 0.0;
    public double flywheelVoltage = 0.0;
    public double flywheelTempC = 0.0;
    public double flywheelVelocity = 0.0;
  }

  public abstract ShooterIOInputsAutoLogged updateInputs();

  public default void setFlywheelVoltage(final double voltage) {} // set voltage

  public default void setFlywheelVelocity(final double rps) {} // set speed

  public default void setPivotVoltage(final double voltage) {} // set speed

  public default void setPivotSetpoint(final Rotation2d rotation) {} // set specfic rotation

  public default void resetPivotPostion(final Rotation2d rotation) {} // reset specfic rotation
}
