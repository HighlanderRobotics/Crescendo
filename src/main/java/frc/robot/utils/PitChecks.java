// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class PitChecks {

  public static Command runCheck(
      Supplier<double[]> expectedValues,
      DoubleSupplier tolerance,
      Supplier<double[]> outputValues,
      Command cmd,
      double time,
      String name) {
    return cmd.withTimeout(time * 2)
        .beforeStarting(() -> Logger.recordOutput(name, "ffff00"))
        .alongWith(
            Commands.waitSeconds(time)
                .finallyDo(
                    () -> {
                      for (int i = 0; i < expectedValues.get().length; i++) { // assumes it's the same length
                        if (MathUtil.isNear(
                            expectedValues.get()[i],
                            outputValues.get()[i],
                            tolerance.getAsDouble())) {
                          Logger.recordOutput(name, "00ff00");
                        } else {
                          Logger.recordOutput(name, "ff0000");
                        }
                      }
                    }));
  }
}