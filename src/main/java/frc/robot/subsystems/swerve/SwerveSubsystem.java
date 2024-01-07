// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import com.google.common.collect.Streams;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import java.util.Arrays;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  // Drivebase constants
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  // Hardware constants
  public static final int PigeonID = 0;

  public static final ModuleConstants frontLeft =
      new ModuleConstants("Front Left", 0, 1, 0, Rotation2d.fromRotations(0.0));
  public static final ModuleConstants frontRight =
      new ModuleConstants("Front Right", 2, 3, 1, Rotation2d.fromRotations(0.0));
  public static final ModuleConstants backLeft =
      new ModuleConstants("Back Left", 4, 5, 2, Rotation2d.fromRotations(0.0));
  public static final ModuleConstants backRight =
      new ModuleConstants("Back Right", 6, 7, 3, Rotation2d.fromRotations(0.0));

  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  private final VisionIO visionIO;
  //private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();
  private AprilTagFieldLayout tagFieldLayout;
  public SwerveDrivePoseEstimator estimator;
  Vector<N3> odoStdDevs = VecBuilder.fill(0.3, 0.3, 0.01);
  Vector<N3> visStdDevs = VecBuilder.fill(1.3, 1.3, 3.3);

  public SwerveSubsystem(GyroIO gyroIO, VisionIO visionIO, ModuleIO... moduleIOs) {
    this.gyroIO = gyroIO;
    modules = (Module[]) Arrays.stream(moduleIOs).map(Module::new).toArray();
  }

  /**
   * Constructs an array of swerve module ios corresponding to the real robot.
   *
   * @return The array of swerve module ios.
   */
  public static ModuleIO[] createTalonFXModules() {
    return new ModuleIO[] {
      new ModuleIOTalonFX(frontLeft),
      new ModuleIOTalonFX(frontRight),
      new ModuleIOTalonFX(backLeft),
      new ModuleIOTalonFX(backRight)
    };
  }

  /**
   * Constructs an array of swerve module ios corresponding to a simulated robot.
   *
   * @return The array of swerve module ios.
   */
  public static ModuleIO[] createSimModules() {
    return new ModuleIO[] {
      new ModuleIOSim("FrontLeft"),
      new ModuleIOSim("FrontRight"),
      new ModuleIOSim("BackLeft"),
      new ModuleIOSim("BackRight")
    };
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Swerve/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    int deltaCount =
        Math.min(
            gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE,
            Arrays.stream(modules)
                .map((m) -> m.getPositionDeltas().length)
                .min(Integer::compare)
                .get());
    for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
      // Read wheel deltas from each module
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        wheelDeltas[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex];
      }

      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      var twist = kinematics.toTwist2d(wheelDeltas);
      if (gyroInputs.connected) {
        // If the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample.
        Rotation2d gyroRotation = gyroInputs.odometryYawPositions[deltaIndex];
        twist = new Twist2d(twist.dx, twist.dy, gyroRotation.minus(lastGyroRotation).getRadians());
        lastGyroRotation = gyroRotation;
      }
      // Apply the twist (change since last sample) to the current pose
      pose = pose.exp(twist);
      // estimator.update(lastGyroRotation, null); //FIXME
    }
    Logger.recordOutput("Swerve Pose", pose);

    // visionIO.updateInputs(visionInputs, new Pose3d(pose));
    // Logger.processInputs("Vision", visionInputs);
    // PhotonPipelineResult result =
    //     new PhotonPipelineResult(visionInputs.latency, visionInputs.targets);
    // // result.setTimestampSeconds(visionInputs.timestamp);
    // try {
    //   var estPose =
    //       VisionHelper.update(
    //               result,
    //               tagFieldLayout,
    //               PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //               PoseStrategy.LOWEST_AMBIGUITY)
    //           .get();
    //   var visionPose =
    //       VisionHelper.update(
    //               result,
    //               tagFieldLayout,
    //               PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //               PoseStrategy.LOWEST_AMBIGUITY)
    //           .get()
    //           .estimatedPose;
    //   Logger.recordOutput("Vision Pose", visionPose);
    //   estimator.addVisionMeasurement(
    //       visionPose.toPose2d(),
    //       visionInputs.timestamp,
    //       VisionHelper.findVisionMeasurements(estPose));
    // } catch (NoSuchElementException e) {
    // }

    // From 6995 - untested
    // try {
    //   var estPose =
    //       VisionHelper.update(
    //               result, tagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    // PoseStrategy.LOWEST_AMBIGUITY)
    //           .get();
    // var visionPose =
    //   VisionHelper.update(
    //           result, tagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    // PoseStrategy.LOWEST_AMBIGUITY)
    //       .get()
    //       .estimatedPose;
    // Logger.recordOutput("Vision Pose", visionPose);
    //   VisionMeasurement measurement = new VisionMeasurement(estPose,
    // VisionHelper.findVisionMeasurements(estPose));
    //     while (measurement != null) {
    //         var estimation = measurement.estimation();
    //         var estimatedPose = estimation.estimatedPose;
    //         // Check height of final pose for sanity. Robot should never be more than 0.5 m off
    // the ground.
    //         if (Math.abs(estimatedPose.getZ()) > 0.5) {
    //             continue;
    //         }
    //         // Skip single-tag measurements with too-high ambiguity.
    //         if (estimation.targetsUsed.size() < 2
    //                 &&
    // estimation.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm() >
    // Units.feetToMeters(13)) {
    //             continue;
    //         }
    //         estimator.addVisionMeasurement(
    //                 measurement.estimation().estimatedPose.toPose2d(),
    //                 measurement.estimation().timestampSeconds,
    //                 measurement.confidence());
    //         Logger.recordOutput("Odo + Vision Pose", estimator.getEstimatedPosition());
    //       }
    //     } catch (NoSuchElementException e) {
    //     }
    Logger.recordOutput(
        "Kalman Pose",
        new Pose2d(
            estimator.getEstimatedPosition().getTranslation(),
            Rotation2d.fromDegrees(estimator.getEstimatedPosition().getRotation().getDegrees())));
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public Command runVelocity(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          // Calculate module setpoints
          ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds.get(), 0.02);
          SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

          // Send setpoints to modules
          SwerveModuleState[] optimizedSetpointStates =
              Streams.zip(
                      Arrays.stream(modules),
                      Arrays.stream(setpointStates),
                      (m, s) -> m.runSetpoint(s))
                  .toArray(SwerveModuleState[]::new);

          // Log setpoint states
          Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
          Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
        });
  }

  /** Stops the drive. */
  public Command stop() {
    return runVelocity(ChassisSpeeds::new);
  }

  public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocity(
        () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getRotation()));
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public Command stopWithX() {
    return this.run(
        () -> {
          Rotation2d[] headings =
              (Rotation2d[])
                  Arrays.stream(getModuleTranslations()).map(Translation2d::getAngle).toArray();
          kinematics.resetHeadings(headings);
          stop();
        });
  }

  /** Runs forwards at the commanded voltage. */
  public Command runCharacterizationVolts(double volts) {
    return this.run(() -> Arrays.stream(modules).forEach((mod) -> mod.runCharacterization(volts)));
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states =
        (SwerveModuleState[]) Arrays.stream(modules).map(Module::getState).toArray();
    return states;
  }

  @AutoLogOutput(key = "Odometry/Velocity")
  public ChassisSpeeds getVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.toChassisSpeeds(
            (SwerveModuleState[])
                Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new)),
        getRotation());
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }
}
