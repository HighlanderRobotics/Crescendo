// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autoaim;

import org.checkerframework.checker.units.qual.C;

import com.google.flatbuffers.FlexBuffers.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.wpilibj2.command.Command;

class ShotData{

    double distance;
    double SHOOTER_HEIGHT = 1.98; // meters 

    public ShotData(double distance) {
        this.distance = distance;
    }

    public Command rotateShooterAngle(){
        return new Command() {
            double angle = Math.atan(SHOOTER_HEIGHT/distance);

            
        };

    }
}
