package frc.robot.subsystems.swerve;

import static frc.robot.DriveTrainConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.utils.mapleUtils.CustomMaths.MapleCommonMath;
import frc.robot.utils.mapleUtils.MapleTimeUtils;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class GyroIOSim implements GyroIO {
  public final GyroPhysicsSimulationResults gyroSimResult = new GyroPhysicsSimulationResults();
  public double previousAngularVelocityRadPerSec = gyroSimResult.robotAngularVelocityRadPerSec;
  public Rotation2d currentGyroDriftAmount = new Rotation2d();
  public static final String GYRO_LOG_PATH = "MaplePhysicsSimulation/GyroSim/";

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    final double
        angularVelocityChange =
            Math.abs(
                gyroSimResult.robotAngularVelocityRadPerSec - previousAngularVelocityRadPerSec),
        angularAccelerationMagnitudeRadPerSecSq = angularVelocityChange / Robot.defaultPeriodSecs;
    previousAngularVelocityRadPerSec = gyroSimResult.robotAngularVelocityRadPerSec;
    final double currentTickDriftStdDevRad =
        angularAccelerationMagnitudeRadPerSecSq
                > GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ
            ? angularAccelerationMagnitudeRadPerSecSq
                * SKIDDING_AMOUNT_AT_THRESHOLD_RAD
                / GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ
            : Math.abs(gyroSimResult.robotAngularVelocityRadPerSec)
                * GYRO_DRIFT_IN_1_TICK_Std_Dev_RAD
                / AVERAGE_VELOCITY_RAD_PER_SEC_DURING_TEST;
    currentGyroDriftAmount =
        currentGyroDriftAmount.rotateBy(
            Rotation2d.fromRadians(
                MapleCommonMath.generateRandomNormal(0, currentTickDriftStdDevRad)));

    inputs.connected = gyroSimResult.hasReading;
    var odometryYawPositions = // wouldve been used for default akit async odo
        Arrays.stream(gyroSimResult.odometryYawPositions)
            .map((robotFacing) -> robotFacing.rotateBy(currentGyroDriftAmount))
            .toArray(Rotation2d[]::new);
    inputs.yawPosition = odometryYawPositions[odometryYawPositions.length - 1];
    inputs.yawVelocityRadPerSec = gyroSimResult.robotAngularVelocityRadPerSec;

    Logger.recordOutput(
        GYRO_LOG_PATH + "robot true yaw (deg)",
        gyroSimResult.odometryYawPositions[gyroSimResult.odometryYawPositions.length - 1]
            .getDegrees());
    Logger.recordOutput(
        GYRO_LOG_PATH + "robot power for (Sec)", MapleTimeUtils.getLogTimeSeconds());
    Logger.recordOutput(
        GYRO_LOG_PATH + "imu total drift (Deg)", currentGyroDriftAmount.getDegrees());
    Logger.recordOutput(GYRO_LOG_PATH + "gyro reading yaw (Deg)", inputs.yawPosition.getDegrees());
    Logger.recordOutput(
        GYRO_LOG_PATH + "angular velocity (Deg per Sec)",
        Math.toDegrees(previousAngularVelocityRadPerSec));
    Logger.recordOutput(
        GYRO_LOG_PATH + "gyro angular acc (Deg per Sec^2)",
        Math.toDegrees(angularAccelerationMagnitudeRadPerSecSq));
    Logger.recordOutput(
        GYRO_LOG_PATH + "new drift in current tick Std Dev (Deg)",
        Math.toDegrees(currentTickDriftStdDevRad));
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    // unimplemented for sim
  }

  /*
   * we know that in one minute, or n=(60 / Robot.defaultPeriodSeconds) periods
   * the gyro's drift has standard deviation of NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD
   * sqrt(n) * GYRO_DRIFT_IN_1_TICK_Std_Dev_RAD = NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD
   *  */
  public static final double GYRO_DRIFT_IN_1_TICK_Std_Dev_RAD =
      NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD / Math.sqrt(60.0 / Robot.defaultPeriodSecs);

  public static class GyroPhysicsSimulationResults {
    public double robotAngularVelocityRadPerSec;
    public boolean hasReading;

    public final Rotation2d[] odometryYawPositions = new Rotation2d[SIMULATION_TICKS_IN_1_PERIOD];

    public GyroPhysicsSimulationResults() {
      robotAngularVelocityRadPerSec = 0.0;
      hasReading = false;
      Arrays.fill(odometryYawPositions, new Rotation2d());
    }
  }
}
