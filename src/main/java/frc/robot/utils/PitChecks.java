// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class PitChecks {

  enum TestState {
      // Green
      SUCCESS("00ff00", "Test successful"),
      // Red
      FAILURE("ff0000", "Test failure"),
      // Yellow
      IN_PROGRESS("ffff00", "In progress"),
      // Gray
      UNKNOWN("dbdbdb", "Test not completed");

      final String color;
      final String msg;
      TestState(String color, String msg) {
          this.color = color;
          this.msg = msg;
      }
  }

  public static Command runCheck(
      Supplier<double[]> expectedValues,
      DoubleSupplier tolerance,
      Supplier<double[]> outputValues,
      Command cmd,
      double time,
      String name) {
    return cmd.withTimeout(time * 2)
        .beforeStarting(() -> pushResult(name, TestState.UNKNOWN))
        .alongWith(
            Commands.runOnce(() -> pushResult(name, TestState.IN_PROGRESS)),
            Commands.waitSeconds(time)
                .finallyDo(
                    () -> {
                      for (int i = 0; i < expectedValues.get().length; i++) { // assumes it's the same length
                        if (MathUtil.isNear(
                            expectedValues.get()[i],
                            outputValues.get()[i],
                            tolerance.getAsDouble())) {
                          pushResult(name, TestState.SUCCESS);
                        } else {
                          pushResult(name, TestState.FAILURE);
                        }
                      }
                    }));
  }
  public static Command runCheck(
          Supplier<double[]> expectedValues,
          Supplier<double[]> tolerance,
          Supplier<double[]> outputValues,
          Command cmd,
          double time,
          String name) {
      return cmd.withTimeout(time * 2)
              .beforeStarting(() -> pushResult(name, TestState.UNKNOWN))
              .alongWith(
                      Commands.runOnce(() -> pushResult(name, TestState.IN_PROGRESS)),
                      Commands.waitSeconds(time)
                              .finallyDo(
                                      () -> {
                                          for (int i = 0; i < expectedValues.get().length; i++) { // assumes it's the same length
                                              if (MathUtil.isNear(
                                                      expectedValues.get()[i],
                                                      outputValues.get()[i],
                                                      tolerance.get()[i])) {
                                                  Logger.recordOutput(name, TestState.SUCCESS.msg);
                                              } else {
                                                  Logger.recordOutput(name, TestState.FAILURE.msg);
                                              }
                                            }
                                      }));
  }

  // Accepts if the output is true
  public static Command runCheck(
          BooleanSupplier output,
          Command cmd,
          double time,
          String name) {
      return cmd.withTimeout(time * 2)
              .beforeStarting(() -> pushResult(name, TestState.UNKNOWN))
              .alongWith(
                      Commands.runOnce(() -> pushResult(name, TestState.IN_PROGRESS)),
                      Commands.waitSeconds(time)
                              .finallyDo(
                                      () -> {
                                          if (output.getAsBoolean()) {
                                              pushResult(name, TestState.SUCCESS);
                                          } else {
                                              pushResult(name, TestState.FAILURE);
                                          }
                                      }
                              )
              );
  }


  private static void pushResult(String name, TestState result) {
      SmartDashboard.putString("Pit Checks/" + name, result.color);
      Logger.recordOutput("Pit Checks/" + name, result.msg);
  }
}
