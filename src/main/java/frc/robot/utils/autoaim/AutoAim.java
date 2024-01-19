// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autoaim;

/** Add your docs here. */
public class AutoAim {

  public static final InterpolatingShotTree shotMap = new InterpolatingShotTree();

  public AutoAim() {

    // Examples until we aquire actual data
    for (double i = 0; i < 10; i++) {
      shotMap.put(i, new ShotData(5 * i, 100 * i));
    }
  }
}