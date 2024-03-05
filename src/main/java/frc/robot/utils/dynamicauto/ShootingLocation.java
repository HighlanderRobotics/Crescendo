// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.dynamicauto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class ShootingLocation {
  private Pose2d pose;
  private String name;

  public ShootingLocation(Pose2d pose, String name) {
    this.pose = pose;
    this.name = name;
  }

  public ShootingLocation() {
    this.pose = new Pose2d();
    this.name = "Uninitialized";
  }

  public ShootingLocation(String name) {
    this.name = name;
  }

  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  public Pose2d getPose() {
    return pose;
  }

  public Pose2d getPoseAllianceSpecific() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        return new Pose2d(
            16.5410515 - this.pose.getX(),
            this.pose.getY(),
            this.pose.getRotation().minus(Rotation2d.fromDegrees(180)));
      }
    }
    return this.pose;
  }

  public String getName() {
    return name;
  }

  public boolean equals(ShootingLocation other) {
    return getName().equals(other.getName());
  }
}
