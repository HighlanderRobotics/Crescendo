// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.dynamicauto;

import edu.wpi.first.math.geometry.Pose2d;

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

  public Pose2d getPose() {
    return pose;
  }

  public String getName() {
    return name;
  }
}
