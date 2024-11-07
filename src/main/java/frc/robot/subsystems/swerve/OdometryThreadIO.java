// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.PhoenixOdometryThread.Samples;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalID;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalType;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface OdometryThreadIO {
  public static final int GYRO_MODULE_ID = -1;

  public class OdometryThreadIOInputs implements LoggableInputs {
    List<Samples> sampledStates = List.of();

    @Override
    public void toLog(LogTable table) {
      var timestamps = sampledStates.stream().mapToDouble(Samples::timestamp).toArray();
      Set<Integer> modIds = new HashSet();
      for (int i = 0; i < sampledStates.size(); i++) {
        var sample = sampledStates.get(i);
        for (var signal : sample.values().entrySet()) {
          table.put(
              "Data/" + i + " " + signal.getKey().type().toString() + " " + signal.getKey().modID(),
              signal.getValue());
          modIds.add(signal.getKey().modID());
        }
      }
      // Remove Gyro/placeholder
      modIds.remove(-1);
      table.put("Module IDs", modIds.stream().mapToInt(Integer::intValue).toArray());
      table.put("Timestamps", timestamps);
    }

    @Override
    public void fromLog(LogTable table) {
      sampledStates = List.of();
      var modIds = table.get("Module IDs").getIntegerArray();
      var timestamps = table.get("Timestamps").getDoubleArray();
      for (int i = 0; i < sampledStates.size(); i++) {
        var timestamp = timestamps[i];
        try {
          Map<SignalID, Double> values = new HashMap();
          for (var id : modIds) {
            values.put(
                new SignalID(SignalType.DRIVE, (int) id),
                table.get("Data/" + i + " " + SignalType.DRIVE + " " + id).getDouble());
            values.put(
                new SignalID(SignalType.STEER, (int) id),
                table.get("Data/" + i + " " + SignalType.STEER + " " + id).getDouble());
          }
          try {
            values.put(
                new SignalID(SignalType.GYRO, (int) -1),
                table.get("Data/" + i + " " + SignalType.GYRO + " " + GYRO_MODULE_ID).getDouble());
          } catch (NullPointerException e) {
            // We don't have a gyro this loop ig
          }
          sampledStates.add(new Samples(timestamp, values));
        } catch (NullPointerException e) {
          System.out.println("Failed to get all swerve signals at " + timestamp);
        }
      }
    }
  }

  public void updateInputs(OdometryThreadIOInputs inputs, double lastTimestamp);

  public void start();

  public void interrupt();
}
