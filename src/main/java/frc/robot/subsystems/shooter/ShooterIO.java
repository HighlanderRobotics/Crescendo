// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.logging.TalonFXLogger.TalonFXLog;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public TalonFXLog pivot = new TalonFXLog(0, 0, 0, 0, 0, 0);

    public TalonFXLog leftFlywheel = new TalonFXLog(0, 0, 0, 0, 0, 0);

    public TalonFXLog rightFlywheel = new TalonFXLog(0, 0, 0, 0, 0, 0);
  }

  public void updateInputs(final ShooterIOInputsAutoLogged inputs);

  public void setFlywheelVoltage(final double left, final double right);

  public void setFlywheelVelocity(final double left, final double right);

  public void setPivotVoltage(final double voltage);

  public void setPivotSetpoint(final Rotation2d rotation);

  public void resetPivotPosition(final Rotation2d rotation);
}
