package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.Module;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * stores the constants and PID configs for chassis. because we want an all-real simulation for the
 * chassis, the numbers are required to be considerably precise
 *
 * <p>Some of these configs are duplicates of ones in Module.java and SwerveSubsystem.java, but
 * there should be a single source of truth for all of them
 */
public class DriveTrainConstants {
  public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.4,
      ROBOT_MASS_KG = 65; // with battery + bumpers
  public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1),
      STEER_MOTOR = DCMotor.getKrakenX60Foc(1);
  public static final double WHEEL_RADIUS_METERS = Module.WHEEL_RADIUS_METERS,
      DRIVE_GEAR_RATIO = Module.DRIVE_GEAR_RATIO,
      STEER_GEAR_RATIO = Module.TURN_GEAR_RATIO,
      TIME_ROBOT_STOP_ROTATING_SECONDS = 0.06,
      STEER_FRICTION_VOLTAGE = 0.28,
      DRIVE_FRICTION_VOLTAGE = 0.04,
      DRIVE_INERTIA = 0.01,
      STEER_INERTIA = 0.01;

  public static final double DRIVE_CURRENT_LIMIT = 120;
  public static final double STEER_CURRENT_LIMIT = Module.TURN_STATOR_CURRENT_LIMIT;

  /** translations of the modules to the robot center, in FL, FR, BL, BR */
  public static final Translation2d[] MODULE_TRANSLATIONS = SwerveSubsystem.getModuleTranslations();

  /* equations that calculates some constants for the simulator (don't modify) */
  private static final double GRAVITY_CONSTANT = 9.8;
  public static final double DRIVE_BASE_RADIUS = MODULE_TRANSLATIONS[0].getNorm(),
      /* friction_force = normal_force * coefficient_of_friction */
      MAX_FRICTION_ACCELERATION = GRAVITY_CONSTANT * WHEEL_COEFFICIENT_OF_FRICTION,
      MAX_FRICTION_FORCE_PER_MODULE =
          MAX_FRICTION_ACCELERATION * ROBOT_MASS_KG / MODULE_TRANSLATIONS.length,
      /* force = torque / distance */
      MAX_PROPELLING_FORCE_NEWTONS =
          DRIVE_MOTOR.getTorque(DRIVE_CURRENT_LIMIT) * DRIVE_GEAR_RATIO / WHEEL_RADIUS_METERS,
      /* floor_speed = wheel_angular_velocity * wheel_radius */
      CHASSIS_MAX_VELOCITY =
          DRIVE_MOTOR.freeSpeedRadPerSec
              / DRIVE_GEAR_RATIO
              * WHEEL_RADIUS_METERS, // calculate using choreo
      CHASSIS_MAX_ACCELERATION_MPS_SQ =
          Math.min(
              MAX_FRICTION_ACCELERATION, // cannot exceed max friction
              MAX_PROPELLING_FORCE_NEWTONS / ROBOT_MASS_KG),
      CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = CHASSIS_MAX_VELOCITY / DRIVE_BASE_RADIUS,
      CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ =
          CHASSIS_MAX_ACCELERATION_MPS_SQ / DRIVE_BASE_RADIUS,
      CHASSIS_FRICTIONAL_ANGULAR_ACCELERATION =
          CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC / TIME_ROBOT_STOP_ROTATING_SECONDS;

  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  /* for collision detection in simulation */
  public static final double BUMPER_WIDTH_METERS = Units.inchesToMeters(35.3),
      BUMPER_LENGTH_METERS = Units.inchesToMeters(39.8),
      /* https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction */
      BUMPER_COEFFICIENT_OF_FRICTION = 0.75,
      /* https://simple.wikipedia.org/wiki/Coefficient_of_restitution */
      BUMPER_COEFFICIENT_OF_RESTITUTION = 0.08;

  /* Gyro Sim */
  public static final double GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ = 100;
  public static final double SKIDDING_AMOUNT_AT_THRESHOLD_RAD = Math.toRadians(1.2);
  /*
   * https://store.ctr-electronics.com/pigeon-2/
   * for a well-installed one with vibration reduction, only 0.4 degree
   * but most teams just install it directly on the rigid chassis frame (including my team :D)
   * so at least 1.2 degrees of drifting in 1 minutes for an average angular velocity of 60 degrees/second
   * which is the average velocity during normal swerve-circular-offense
   * */
  public static final double NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD = Math.toRadians(1.2);
  public static final double AVERAGE_VELOCITY_RAD_PER_SEC_DURING_TEST = Math.toRadians(60);

  /* dead configs, don't change them */
  public static final int ODOMETRY_CACHE_CAPACITY = 10;
  public static final double ODOMETRY_FREQUENCY = 250;
  public static final double ODOMETRY_WAIT_TIMEOUT_SECONDS = 0.02;
  public static final int SIMULATION_TICKS_IN_1_PERIOD = 5;
}
