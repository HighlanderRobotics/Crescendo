// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autoaim;

/** Add your docs here. */
public class AutoAim {

  public static final InterpolatingShotTree speakerShotMap = new InterpolatingShotTree();

  public static final InterpolatingShotTree ampShotMap = new InterpolatingShotTree();

  public AutoAim() {

    // Examples until we aquire actual data
    for (double i = 0; i < 10; i++) {
      speakerShotMap.put(i, new ShotData(5 * i, 100 * i, 0.1 * i));
    }

    for (double i = 0; i < 10; i++) {
      ampShotMap.put(i, new ShotData(2.3 * i, 100 * i, 0.05 * i));
    }
  }
}
