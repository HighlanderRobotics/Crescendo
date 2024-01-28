package frc.robot.utils.components;

import edu.wpi.first.wpilibj.DigitalInput;

/** A digital input (ie a beambreak or limit switch) whose normal state can be reversed */
public class ReversibleDigitalInput {
  final boolean reversed;
  final DigitalInput input;

  public ReversibleDigitalInput(int channel, boolean reversed) {
    input = new DigitalInput(channel);
    this.reversed = reversed;
  }

  public boolean get() {
    return input.get() != reversed;
  }
}