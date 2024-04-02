// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

/** A CommandXboxController that implements Subsystem to allow the rumble to be mutexed. */
public class CommandXboxControllerSubsystem extends CommandXboxController implements Subsystem {

  public CommandXboxControllerSubsystem(int port) {
    super(port);
  }

  /** Rumble the controller at the specified power. */
  public Command rumbleCmd(DoubleSupplier left, DoubleSupplier right) {
    return this.run(
            () -> {
              super.getHID().setRumble(RumbleType.kLeftRumble, left.getAsDouble());
              super.getHID().setRumble(RumbleType.kRightRumble, right.getAsDouble());
            })
        .finallyDo(() -> super.getHID().setRumble(RumbleType.kBothRumble, 0.0));
  }

  /** Rumble the controller at the specified power. */
  public Command rumbleCmd(double left, double right) {
    return rumbleCmd(() -> left, () -> right);
  }
}
