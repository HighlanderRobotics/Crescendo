// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
/** Add your docs here. */
public interface ShooterIO {
    public static class ShooterIOInputs {
     public double PivotAmps = 0;
     public double PivotMeters = 0.0;
     public double PivotVoltage = 0.0;
     public double PivotAmp = 0.0;
     public double PivotTempc = 0.0;
     public double PivotVelocity = 0.0;
     public Rotation2d turnPosition = new Rotation2d();
     public double FlywheelAmps = 0;
     public double FlywheelVoltage = 0.0;
     public double FlywheelAmp = 0.0;
     public double FlywheelTempc = 0.0;
     public double FlywheelVelocity = 0.0;
   

    }


}








