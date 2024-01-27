// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class AutoAim {

  public static final InterpolatingShotTree shotMap = new InterpolatingShotTree();

  public static final double LOOKAHEAD_TIME = 1.0;

  static {

    // Examples until we aquire actual data
    for (double i = 0; i < 10; i++) {
      shotMap.put(i, new ShotData(Rotation2d.fromDegrees(5 * i), 100 * i, 0.1 * i));
    }
  }
}
