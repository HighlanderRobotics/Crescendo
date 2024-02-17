// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

// This is a copy of LEDIOReal right now, but if we move leds to separate ports sim breaks w ledioreal
public class LEDIOSim implements LEDIO {
  AddressableLED led;
  AddressableLEDBuffer buffer;

  public LEDIOSim() {
    led = new AddressableLED(3);
    buffer = new AddressableLEDBuffer(LEDSubsystem.LED_LENGTH);
    led.setLength(buffer.getLength());
    led.start();
  }

  /** Write data from buffer to leds */
  @Override
  public void updateInputs(LEDIOInputs inputs) {
    led.setData(buffer);
  }

  @Override
  public void set(int i, Color color) {
    buffer.setLED(i, color);
  }
}
