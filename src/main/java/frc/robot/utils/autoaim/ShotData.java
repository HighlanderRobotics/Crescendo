// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Rotation2d;

/*
 * Holds data about each shot in which we interpolate with
 */
public class ShotData {

  private final Rotation2d angle;
  private final double leftRotationsPerSecond;
  private final double rightRotationsPerSecond;
  private final double flightTimeSeconds;

  public ShotData(
      Rotation2d angle,
      double leftRotationsPerSecond,
      double rightRotationsPerSecond,
      double flightTime) {
    this.angle = angle;
    this.leftRotationsPerSecond = leftRotationsPerSecond;
    this.rightRotationsPerSecond = rightRotationsPerSecond;
    this.flightTimeSeconds = flightTime;
  }

  public Rotation2d getAngle() {
    return angle;
  }

  public double getLeftRPM() {
    return leftRotationsPerSecond;
  }

  public double getRightRPM() {
    return rightRotationsPerSecond;
  }

  public double getFlightTime() {
    return flightTimeSeconds;
  }

  public String toString() {
    return "" + getAngle() + " " + getLeftRPM() + " " + getRightRPM() + " " + getFlightTime();
  }
}
