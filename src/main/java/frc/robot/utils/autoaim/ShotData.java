// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLogOutput;

/*
 * Holds data about each shot in which we interpolate with
 */
public class ShotData {

  private Rotation2d angle;
  private double rotationsPerSecond;
  private double flightTimeSeconds;

  public ShotData(Rotation2d angle, double rotationsPerSecond, double flightTime) {
    this.angle = angle;
    this.rotationsPerSecond = rotationsPerSecond;
    this.flightTimeSeconds = flightTime;
  }


  public Rotation2d getAngle() {
    return angle;
  }


  public double getRPM() {
    return rotationsPerSecond;
  }


  public double getFlightTime() {
    return flightTimeSeconds;
  }

  public String toString() {
    return "" + getAngle() + " " + getRPM() + " " + getFlightTime();
  }
}
