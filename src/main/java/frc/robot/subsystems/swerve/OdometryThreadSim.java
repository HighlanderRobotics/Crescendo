package frc.robot.subsystems.swerve;

import static frc.robot.DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.*;
import frc.robot.utils.mapleUtils.MapleTimeUtils;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public final class OdometryThreadSim implements OdometryThreadIO {
  final ModuleIOSim[] modules;
  final GyroIOSim gyro;

  public OdometryThreadSim(ModuleIOSim[] modules, GyroIOSim gyro) {
    this.modules = modules;
    this.gyro = gyro;
  }

  @Override
  public void updateInputs(OdometryThreadIOInputs inputs, double lastTimestamp) {
    final double robotStartingTimeStamps = MapleTimeUtils.getLogTimeSeconds(),
        iterationPeriodSeconds = Robot.defaultPeriodSecs / SIMULATION_TICKS_IN_1_PERIOD;

    inputs.sampledStates = new ArrayList<>();
    for (int i = 0; i < SIMULATION_TICKS_IN_1_PERIOD; i++) {
      var timestamp = robotStartingTimeStamps + i * iterationPeriodSeconds;
      try {
        Map<SignalID, Double> values = new HashMap();
        for (int id = 0; id < 4; id++) {
          values.put(
              new SignalID(SignalType.DRIVE, (int) id),
              modules[id].driveSimResults.odometryDriveWheelRevolutions[i]);
          values.put(
              new SignalID(SignalType.STEER, (int) id),
              modules[id].driveSimResults.odometrySteerPositions[i].getRotations());
        }
        try {
          values.put(
              new SignalID(SignalType.GYRO, (int) -1),
              gyro.gyroSimResult.odometryYawPositions[i].getDegrees());
        } catch (NullPointerException e) {
          // We don't have a gyro this loop ig
        }
        inputs.sampledStates.add(new Samples(timestamp, values));
      } catch (NullPointerException e) {
        System.out.println("Failed to get all swerve signals at " + timestamp);
      }
    }
  }

  @Override
  public void start() {}

  @Override
  public void interrupt() {}
}
