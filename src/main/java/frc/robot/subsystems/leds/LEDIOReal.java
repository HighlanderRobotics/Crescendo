// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIOReal implements LEDIO {
  AddressableLED leftLed;
  AddressableLED rightLed;
  AddressableLEDBuffer buffer;

  public LEDIOReal() {
    leftLed = new AddressableLED(3);
    rightLed = new AddressableLED(4);
    buffer = new AddressableLEDBuffer(LEDSubsystem.LED_LENGTH);
    leftLed.setLength(buffer.getLength());
    leftLed.start();
    rightLed.setLength(buffer.getLength());
    rightLed.start();
  }

  /** Write data from buffer to leds */
  @Override
  public void updateInputs(LEDIOInputs inputs) {
    leftLed.setData(buffer);
    rightLed.setData(buffer);
  }

  @Override
  public void set(int i, Color color) {
    buffer.setLED(i, color);
  }
}
