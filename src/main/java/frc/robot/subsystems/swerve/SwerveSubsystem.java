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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.google.common.collect.Streams;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.utils.autoaim.AutoAim;

public class SwerveSubsystem extends SubsystemBase {
  // Drivebase constants
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(20.5);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.5);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  // Hardware constants
  public static final int PIGEON_ID = 0;

  public static final ModuleConstants frontLeft =
      new ModuleConstants("Front Left", 6, 5, 21, Rotation2d.fromRotations(0.125732));
  public static final ModuleConstants frontRight =
      new ModuleConstants("Front Right", 8, 7, 23, Rotation2d.fromRotations(0.461426));
  public static final ModuleConstants backLeft =
      new ModuleConstants("Back Left", 4, 3, 20, Rotation2d.fromRotations(0.152344));
  public static final ModuleConstants backRight =
      new ModuleConstants("Back Right", 2, 1, 22, Rotation2d.fromRotations(-0.238281));

  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Pose2d pose = new Pose2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d rawGyroRotation = new Rotation2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  private final Vision[] cameras;
  public static final AprilTagFieldLayout fieldTags =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  public SwerveDrivePoseEstimator estimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, pose);
  Vector<N3> odoStdDevs = VecBuilder.fill(0.3, 0.3, 0.01);
  Vector<N3> visStdDevs = VecBuilder.fill(1.3, 1.3, 3.3);
  private double lastEstTimestamp = 0;

  public static final Matrix<N3, N3> LEFT_CAMERA_MATRIX =
      MatBuilder.fill(
          Nat.N3(),
          Nat.N3(),
          923.5403619629557,
          0.0,
          644.4965658066068,
          0.0,
          925.8136962361125,
          402.6412935350414,
          0.0,
          0.0,
          1.0); // TODO find!!
  public static final Matrix<N5, N1> LEFT_DIST_COEFFS =
      MatBuilder.fill(
          Nat.N5(),
          Nat.N1(),
          0.05452153950284706,
          -0.04331612051891956,
          0.00176988756858703,
          -0.004530368741385627,
          -0.040501622476628085); // TODO find!!
  public static final Matrix<N3, N3> RIGHT_CAMERA_MATRIX_OPT =
      MatBuilder.fill(
          Nat.N3(),
          Nat.N3(),
          923.5403619629557,
          0.0,
          644.4965658066068,
          0.0,
          925.8136962361125,
          402.6412935350414,
          0.0,
          0.0,
          1.0); // TODO find!!
  public static final Matrix<N5, N1> RIGHT_DIST_COEFFS_OPT =
      MatBuilder.fill(
          Nat.N5(),
          Nat.N1(),
          0.05452153950284706,
          -0.04331612051891956,
          0.00176988756858703,
          -0.004530368741385627,
          -0.040501622476628085); // TODO find!!
  public static final VisionConstants leftCamConstants =
      new VisionConstants(
          "Left Camera",
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(14.4), Units.inchesToMeters(0), Units.inchesToMeters(29.75)),
              new Rotation3d(0, 0, 0)),
          new VisionSystemSim("Left Camera Sim System"),
          LEFT_CAMERA_MATRIX,
          LEFT_DIST_COEFFS); // TODO this *should* be the transform on the alpha bot but is probably
  // wrong
  public static final VisionConstants rightCamConstants =
      new VisionConstants(
          "Right Camera",
          new Transform3d(),
          new VisionSystemSim("Right Camera Sim System"),
          RIGHT_CAMERA_MATRIX_OPT,
          RIGHT_DIST_COEFFS_OPT); // TODO find transforms
  private SwerveDriveOdometry odometry;

  public SwerveSubsystem(GyroIO gyroIO, VisionIO[] visionIOs, ModuleIO[] moduleIOs) {
    this.gyroIO = gyroIO;
    cameras = new Vision[visionIOs.length];
    new AutoAim();
    modules = new Module[moduleIOs.length];

    for (int i = 0; i < moduleIOs.length; i++) {
      modules[i] = new Module(moduleIOs[i]);
    }
    for (int i = 0; i < visionIOs.length; i++) {
      cameras[i] = new Vision(visionIOs[i]);
    }

    // mildly questionable
    VisionIOSim.pose = this::getPose3d;

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            MAX_LINEAR_SPEED, // Max module speed, in m/s
            DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig(
                false, false) // Default path replanning config. See the API for the options
            // here
            ),
        () -> false,
        this // Reference to this subsystem to set requirements
        );

    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          Logger.recordOutput("PathPlanner/Target", pose);
          Logger.recordOutput(
              "PathPlanner/Absolute Translation Error",
              pose.minus(getPose()).getTranslation().getNorm());
        });
    PathPlannerLogging.setLogActivePathCallback(
        (path) -> Logger.recordOutput("PathPlanner/Active Path", path.toArray(Pose2d[]::new)));
    Logger.recordOutput("PathPlanner/Target", new Pose2d());
    Logger.recordOutput("PathPlanner/Absolute Translation Error", 0.0);

    odometry = new SwerveDriveOdometry(kinematics, getRotation(), getModulePositions());
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

  /**
   * Constructs an array of vision IOs corresponding to the real robot.
   *
   * @return The array of vision IOs.
   */
  public static VisionIO[] createRealCameras() {
    return new VisionIO[] {new VisionIOReal(leftCamConstants), new VisionIOReal(rightCamConstants)};
  }

  /**
   * Constructs an array of vision IOs corresponding to the simulated robot.
   *
   * @return The array of vision IOs.
   */
  public static VisionIO[] createSimCameras() {
    return new VisionIO[] {new VisionIOSim(leftCamConstants), new VisionIOSim(rightCamConstants)};
  }

  public void periodic() {
    for (var camera : cameras) {
      camera.updateInputs();
      camera.processInputs();
    }
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
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int deltaIndex = 0; deltaIndex < sampleCount; deltaIndex++) {
      // Read wheel deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      SwerveModulePosition[] estPositions = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[deltaIndex];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        estPositions[moduleIndex] = //TODO i don't know why this works but it does 
            new SwerveModulePosition(
                getModulePositions()[moduleIndex].distanceMeters //smth abt odo positions vs module positions
                    - lastModulePositions[moduleIndex].distanceMeters,
                getModulePositions()[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      Twist2d twist = kinematics.toTwist2d(modulePositions);
      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[deltaIndex];
        twist =
            new Twist2d(twist.dx, twist.dy, rawGyroRotation.minus(lastGyroRotation).getRadians());
      } else {
        // Use the angle delta from the kinematics and module deltas
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      pose = pose.exp(twist);
      lastGyroRotation = rawGyroRotation;
      // Apply update
      estimator.updateWithTime(sampleTimestamps[deltaIndex], rawGyroRotation, estPositions);
    }

    List<Pose3d> visionPoses = new ArrayList<>();
    for (var camera : cameras) {
      PhotonPipelineResult result =
          new PhotonPipelineResult(camera.inputs.latency, camera.inputs.targets);
      result.setTimestampSeconds(camera.inputs.timestamp);
      boolean newResult = Math.abs(camera.inputs.timestamp - lastEstTimestamp) > 1e-5;
      try {
        var estPose = camera.update(result);
        var visionPose = estPose.get().estimatedPose;
        // Sets the pose on the sim field
        camera.setSimPose(estPose, camera, newResult);
        visionPoses.add(visionPose);
        Logger.recordOutput("Vision/Vision Pose From " + camera.getName(), visionPose);
        // estimator.addVisionMeasurement(
        //     visionPose.toPose2d(),
        //     camera.inputs.timestamp,
        //     VisionHelper.findVisionMeasurementStdDevs(estPose.get()));
        if (newResult) lastEstTimestamp = camera.inputs.timestamp;
      } catch (NoSuchElementException e) {
      }
    }
    Logger.recordOutput(
        "Kalman Pose",
        new Pose2d(
            estimator.getEstimatedPosition().getTranslation(),
            Rotation2d.fromDegrees(estimator.getEstimatedPosition().getRotation().getDegrees())));
  }

  private void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    Logger.recordOutput("Swerve/Target Speeds", discreteSpeeds);
    Logger.recordOutput("Swerve/Speed Error", discreteSpeeds.minus(getVelocity()));

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates =
        Streams.zip(
                Arrays.stream(modules), Arrays.stream(setpointStates), (m, s) -> m.runSetpoint(s))
            .toArray(SwerveModuleState[]::new);

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public Command runVelocityCmd(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> runVelocity(speeds.get()));
  }

  /** Stops the drive. */
  public Command stopCmd() {
    return runVelocityCmd(ChassisSpeeds::new);
  }

  public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocityCmd(
        () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getRotation()));
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public Command stopWithXCmd() {
    return this.run(
        () -> {
          Rotation2d[] headings = new Rotation2d[4];
          for (int i = 0; i < modules.length; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
          }
          kinematics.resetHeadings(headings);
          stopCmd();
        });
  }

  /** Runs forwards at the commanded voltage. */
  public Command runCharacterizationVoltsCmd(double volts) {
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

    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    return states;
  }

  @AutoLogOutput(key = "Odometry/Velocity")
  public ChassisSpeeds getVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new)),
        getRotation());
  }

  @AutoLogOutput(key = "Odometry/RobotRelativeVelocity")
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        (SwerveModuleState[])
            Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new));
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  public Pose3d getPose3d() {
    return new Pose3d(pose);
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  /** Sets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.pose = pose;
    try {
      estimator.resetPosition(pose.getRotation(), lastModulePositions, pose);
    } catch (Exception e) {
    }
    odometry.resetPosition(gyroInputs.yawPosition, getModulePositions(), pose);
  }

  public void setYaw(Rotation2d yaw) {
    gyroIO.setYaw(yaw);
    setPose(new Pose2d(getPose().getTranslation(), yaw));
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

  public static VisionConstants[] getCameraConstants() {
    return new VisionConstants[] {leftCamConstants, rightCamConstants};
  }

  /** Returns the module positions (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  public Rotation2d getRotationToTranslation(Translation2d translation) {

    double angle = Math.atan2(translation.getY() - pose.getY(), translation.getX() - pose.getX());
    return Rotation2d.fromRadians(angle);
  }

  public Command pointTowardsTranslation(DoubleSupplier x, DoubleSupplier y) {
    ProfiledPIDController headingController =
        // assume we can accelerate to max in 0.5 seconds
        new ProfiledPIDController(
            1.0, 0.0, 0.0, new Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED / 0.5));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return this.runVelocityFieldRelative(
            () -> {
              double calculated =
                  headingController.calculate(
                      getPose().getRotation().getRadians(),
                      getRotationToTranslation(FieldConstants.getSpeaker()).getRadians());

              return new ChassisSpeeds(
                  x.getAsDouble(),
                  y.getAsDouble(),
                  calculated + headingController.getSetpoint().velocity);
            })
        .beforeStarting(
            () ->
                headingController.reset(
                    new State(
                        getPose().getRotation().getRadians(),
                        getVelocity().omegaRadiansPerSecond)));
  }
}
