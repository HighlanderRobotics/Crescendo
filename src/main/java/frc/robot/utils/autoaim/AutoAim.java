// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class AutoAim {

  public static final InterpolatingShotTree shotMap = new InterpolatingShotTree();

  public static final double LOOKAHEAD_TIME_SECONDS = 1.0;

  static {
    shotMap.put(1.0, new ShotData(new Rotation2d(55), 60, 80, 0.25));
    shotMap.put(2.0, new ShotData(new Rotation2d(45), 60, 80, 0.31));
    shotMap.put(3.0, new ShotData(new Rotation2d(34), 60, 80, 0.36));
    shotMap.put(4.0, new ShotData(new Rotation2d(30), 70, 90, 0.4));
  }
}
