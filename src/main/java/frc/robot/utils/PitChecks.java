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

  enum TestResult {
      SUCCESS("00ff00", "Test successful"),
      FAILURE("ff0000", "Test failure"),
      UNKNOWN("ffff00", "Test not completed");

      final String color;
      final String msg;
      TestResult(String color, String msg) {
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
        .beforeStarting(() -> pushResult(name, TestResult.UNKNOWN))
        .alongWith(
            Commands.waitSeconds(time)
                .finallyDo(
                    () -> {
                      for (int i = 0; i < expectedValues.get().length; i++) { // assumes it's the same length
                        if (MathUtil.isNear(
                            expectedValues.get()[i],
                            outputValues.get()[i],
                            tolerance.getAsDouble())) {
                          pushResult(name, TestResult.SUCCESS);
                        } else {
                          pushResult(name, TestResult.FAILURE);
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
              .beforeStarting(() -> pushResult(name, TestResult.UNKNOWN))
              .alongWith(
                      Commands.waitSeconds(time)
                              .finallyDo(
                                      () -> {
                                          for (int i = 0; i < expectedValues.get().length; i++) { // assumes it's the same length
                                              if (MathUtil.isNear(
                                                      expectedValues.get()[i],
                                                      outputValues.get()[i],
                                                      tolerance.get()[i])) {
                                                  Logger.recordOutput(name, TestResult.SUCCESS.msg);
                                              } else {
                                                  Logger.recordOutput(name, TestResult.FAILURE.msg);
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
              .beforeStarting(() -> pushResult(name, TestResult.UNKNOWN))
              .alongWith(
                      Commands.waitSeconds(time)
                              .finallyDo(
                                      () -> {
                                          if (output.getAsBoolean()) {
                                              pushResult(name, TestResult.SUCCESS);
                                          } else {
                                              pushResult(name, TestResult.FAILURE);
                                          }
                                      }
                              )
              );
  }


  private static void pushResult(String name, TestResult result) {
      SmartDashboard.putString(name, result.color);
      Logger.recordOutput(name, result.msg);
  }
}
