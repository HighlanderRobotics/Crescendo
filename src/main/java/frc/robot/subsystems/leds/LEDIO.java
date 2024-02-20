// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public static class LEDIOInputs {
    // No sensors so no inputs
  }

  public void updateInputs(LEDIOInputs inputs);

  public void set(int i, Color color);

  public default void solid(Color color) {
    for (int i = 0; i < LEDSubsystem.LED_LENGTH; i++) {
      set(i, color);
    }
  }
}
