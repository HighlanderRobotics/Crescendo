// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class ShotAngle implements Comparable<ShotAngle> {

    Rotation2d rotation;

    public ShotAngle(Rotation2d rotation) {
        this.rotation = rotation;
    }

    @Override
    public int compareTo(ShotAngle o) {
        return Double.valueOf(rotation.getRotations()).compareTo(o.rotation.getRotations());
    }
}

// class ShotPose implements Comparable<ShotPose> {

//     Pose2d pose;

//     public ShotPose(Pose2d pose) {
//         this.pose = pose;
//     }

//     @Override
//     public int compareTo(ShotPose o) {
//         return Double.valueOf(pose.getRotations()).compareTo(o.pose.getRotations());
//     }
// }
