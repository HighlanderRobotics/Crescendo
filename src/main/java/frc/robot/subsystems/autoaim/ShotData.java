// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autoaim;

import edu.wpi.first.math.interpolation.Interpolatable;

public class ShotData implements Interpolatable<ShotData> {

  private double angle;
  private double rotationsPerSecond;

  public ShotData(double angle, double rotationsPerSecond) {
    this.angle = angle;
    this.rotationsPerSecond = rotationsPerSecond;
  }

  public double getAngle() {
    return angle;
  }

  public double getRPM() {
    return rotationsPerSecond;
  }

  @Override
  public ShotData interpolate(ShotData endValue, double t) {
    return new ShotData(
        ((endValue.getAngle() - angle) * t) + angle,
        ((endValue.getRPM() - rotationsPerSecond) * t) + rotationsPerSecond);
  }

  public String toString() {
    return "" + getAngle() + " " + getRPM();
  }
}
