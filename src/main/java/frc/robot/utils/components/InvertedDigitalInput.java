package frc.robot.utils.components;

import edu.wpi.first.wpilibj.DigitalInput;

/** A digital input (ie a beambreak or limit switch) whose normal state is reversed */
public class InvertedDigitalInput extends DigitalInput {
  private InvertedDigitalInput(int channel) {
    super(channel);
  }

  public static InvertedDigitalInput from(final DigitalInput input) {
    return new InvertedDigitalInput(input.getChannel());
  }

  public boolean get() {
    return !super.get();
  }
}
