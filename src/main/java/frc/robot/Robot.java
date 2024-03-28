// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.carriage.CarriageIOReal;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDIOReal;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Samples;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.AutoAimStates;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.autoaim.AutoAim;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public static enum RobotMode {
    SIM,
    REPLAY,
    REAL
  }

  public static enum Target {
    AMP,
    SPEAKER
  }

  public static final RobotMode mode = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;
  public static final boolean USE_AUTO_AIM = true;
  public static final boolean USE_SOTM = false;
  private Command autonomousCommand;
  private ChoreoTrajectory activeChoreoTrajectory;
  private LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  private LoggedDashboardNumber dashShotDegrees =
      new LoggedDashboardNumber("Rotation (degrees)", 37.0);
  private LoggedDashboardNumber dashShotLeftRPS =
      new LoggedDashboardNumber("Left RPS (Rotations Per Sec)", 60.0);
  private LoggedDashboardNumber dashShotRightRPS =
      new LoggedDashboardNumber("Right RPS (Rotations Per Sec)", 80.0);

  private final CommandXboxControllerSubsystem controller = new CommandXboxControllerSubsystem(0);
  private final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  private final SlewRateLimiter vxLimiter =
      new SlewRateLimiter(SwerveSubsystem.MAX_LINEAR_ACCELERATION * 0.9);
  private final SlewRateLimiter vyLimiter =
      new SlewRateLimiter(SwerveSubsystem.MAX_LINEAR_ACCELERATION * 0.9);
  private final SlewRateLimiter omegaLimiter =
      new SlewRateLimiter(SwerveSubsystem.MAX_ANGULAR_ACCELERATION * 0.9);

  private Target currentTarget = Target.SPEAKER;
  private double flywheelIdleSpeed = 30.0;

  private final SwerveSubsystem swerve =
      new SwerveSubsystem(
          mode == RobotMode.REAL
              ? new GyroIOPigeon2()
              : new GyroIO() {
                // Blank implementation to mock gyro in sim
                @Override
                public void updateInputs(GyroIOInputs inputs, List<Samples> asyncOdometrySamples) {}

                @Override
                public void setYaw(Rotation2d yaw) {}
              },
          mode == RobotMode.REAL
              ? SwerveSubsystem.createRealCameras()
              : SwerveSubsystem.createSimCameras(),
          mode == RobotMode.REAL
              ? SwerveSubsystem.createTalonFXModules()
              : SwerveSubsystem.createSimModules());
  private final IntakeSubsystem intake = new IntakeSubsystem(new IntakeIOReal());
  private final FeederSubsystem feeder = new FeederSubsystem(new FeederIOReal());
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(mode == RobotMode.REAL ? new ElevatorIOReal() : new ElevatorIOSim());
  private final ShooterSubsystem shooter =
      new ShooterSubsystem(mode == RobotMode.REAL ? new ShooterIOReal() : new ShooterIOSim());
  private final CarriageSubsystem carriage = new CarriageSubsystem(new CarriageIOReal());
  private final LEDSubsystem leds =
      new LEDSubsystem(mode == RobotMode.REAL ? new LEDIOReal() : new LEDIOSim());

  @Override
  public void robotInit() {
    // Metadata about the current code running on the robot
    Logger.recordMetadata("Codebase", "Comp2024");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("Robot Mode", mode.toString());
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (mode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
    SignalLogger.setPath("/media/sda1/");

    // Default Commands here
    swerve.setDefaultCommand(
        swerve.runVoltageTeleopFieldRelative(
            () ->
                new ChassisSpeeds(
                    -teleopAxisAdjustment(controller.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                    -teleopAxisAdjustment(controller.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                    -teleopAxisAdjustment(controller.getRightX())
                        * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    elevator.setDefaultCommand(elevator.setExtensionCmd(() -> 0.0));
    feeder.setDefaultCommand(
        Commands.repeatingSequence(
            feeder.indexCmd().until(() -> currentTarget != Target.SPEAKER),
            Commands.sequence(
                    feeder
                        .runVelocityCmd(-FeederSubsystem.INDEXING_VELOCITY)
                        .until(() -> carriage.getBeambreak()),
                    feeder.runVelocityCmd(-FeederSubsystem.INDEXING_VELOCITY).withTimeout(0.5),
                    feeder.runVelocityCmd(0))
                .until(() -> currentTarget != Target.AMP)));
    carriage.setDefaultCommand(
        Commands.repeatingSequence(
            Commands.either(
                    carriage.indexBackwardsCmd(),
                    carriage.indexForwardsCmd(),
                    () -> feeder.getFirstBeambreak())
                .until(() -> currentTarget != Target.AMP),
            Commands.repeatingSequence(
                    carriage
                        .runVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE)
                        .until(() -> feeder.getFirstBeambreak()),
                    carriage.runVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE).withTimeout(0.5),
                    carriage.runVoltageCmd(-0.5).until(() -> !feeder.getFirstBeambreak()))
                .until(() -> currentTarget != Target.SPEAKER)));
    intake.setDefaultCommand(intake.runVoltageCmd(0.0, 0.0));
    shooter.setDefaultCommand(shooter.runFlywheelsCmd(() -> 0.0, () -> 0.0));
    leds.setDefaultCommand(
        leds.defaultStateDisplayCmd(
            () -> DriverStation.isEnabled(), () -> currentTarget == Target.SPEAKER));

    controller.setDefaultCommand(controller.rumbleCmd(0.0, 0.0));
    operator.setDefaultCommand(operator.rumbleCmd(0.0, 0.0));

    // Robot state management bindings
    new Trigger(() -> carriage.getBeambreak())
        .debounce(0.5)
        .onTrue(intake.runVelocityCmd(-50.0, -30.0).withTimeout(1.0));
    new Trigger(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
        .debounce(0.25)
        .onTrue(
            Commands.parallel(
                    controller.rumbleCmd(1.0, 1.0),
                    operator.rumbleCmd(1.0, 1.0),
                    leds.setBlinkingCmd(new Color("#ff4400"), new Color("#000000"), 25.0))
                .withTimeout(0.5));
    new Trigger(() -> DriverStation.isEnabled()).onTrue(elevator.unlockClimb());

    // ---- Controller bindings here ----
    // Prevent intaking when elevator isnt down
    controller
        .leftTrigger()
        .and(() -> elevator.getExtensionMeters() < Units.inchesToMeters(2.0))
        .whileTrue(intake.runVelocityCmd(60.0, 30.0));
    controller
        .rightTrigger()
        .and(() -> currentTarget == Target.SPEAKER)
        .and(() -> USE_AUTO_AIM)
        .and(() -> USE_SOTM)
        .whileTrue(teleopAutoAim());
    controller
        .rightTrigger()
        .and(() -> currentTarget == Target.SPEAKER)
        .and(() -> USE_AUTO_AIM)
        .and(() -> !USE_SOTM)
        .whileTrue(staticAutoAim());
    controller
        .rightTrigger()
        .and(() -> currentTarget == Target.SPEAKER)
        .and(operator.b())
        .whileTrue(
            shooter.runStateCmd(
                AutoAim.FEED_SHOT.getRotation(),
                AutoAim.FEED_SHOT.getLeftRPS(),
                AutoAim.FEED_SHOT.getRightRPS()))
        .onFalse(
            Commands.parallel(
                    shooter.run(() -> {}), feeder.runVelocityCmd(FeederSubsystem.INDEXING_VELOCITY))
                .withTimeout(0.5));
    controller
        .rightTrigger()
        .and(() -> currentTarget == Target.SPEAKER)
        .and(() -> !USE_AUTO_AIM)
        .whileTrue(shooter.runStateCmd(Rotation2d.fromDegrees(50.0), 50.0, 60.0))
        .onFalse(
            Commands.parallel(
                    shooter.run(() -> {}), feeder.runVelocityCmd(FeederSubsystem.INDEXING_VELOCITY))
                .withTimeout(0.5));
    controller
        .rightTrigger()
        .and(() -> currentTarget == Target.AMP)
        .and(operator.leftBumper().negate().debounce(0.5))
        .and(operator.leftTrigger().negate().debounce(0.5))
        .whileTrue(elevator.setExtensionCmd(() -> ElevatorSubsystem.AMP_EXTENSION_METERS))
        .onFalse(
            Commands.parallel(
                    carriage.runVoltageCmd(-3.0),
                    elevator.setExtensionCmd(() -> ElevatorSubsystem.AMP_EXTENSION_METERS))
                .withTimeout(0.75));
    controller.leftBumper().whileTrue(swerve.stopWithXCmd());
    controller
        .rightBumper()
        .and(() -> currentTarget == Target.SPEAKER)
        .and(controller.rightTrigger().negate())
        .whileTrue(
            speakerHeadingSnap(
                    () ->
                        -teleopAxisAdjustment(controller.getLeftY())
                            * SwerveSubsystem.MAX_LINEAR_SPEED,
                    () ->
                        -teleopAxisAdjustment(controller.getLeftX())
                            * SwerveSubsystem.MAX_LINEAR_SPEED)
                .until(controller.rightTrigger()));
    controller
        .rightBumper()
        .and(() -> currentTarget == Target.AMP)
        .whileTrue(
            ampHeadingSnap(
                () ->
                    -teleopAxisAdjustment(controller.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                () ->
                    -teleopAxisAdjustment(controller.getLeftX())
                        * SwerveSubsystem.MAX_LINEAR_SPEED));

    controller
        .x()
        .whileTrue(
            Commands.parallel(
                shooter.runFlywheelVoltageCmd(Rotation2d.fromDegrees(30.0), -5.0),
                feeder.runVoltageCmd(-5.0),
                carriage.runVoltageCmd(-5.0),
                intake.runVelocityCmd(-50.0, -50.0)));

    // climb
    operator
        .rightTrigger(0.75)
        .and(() -> elevator.getExtensionMeters() < 0.25)
        .and(operator.rightBumper())
        .onFalse(
            Commands.parallel(
                    elevator.setExtensionCmd(() -> ElevatorSubsystem.CLIMB_EXTENSION_METERS),
                    leds.setBlinkingCmd(new Color("#00ff00"), new Color("#ffffff"), 10.0))
                .until(
                    () ->
                        operator.getRightTriggerAxis() > 0.75
                            && operator.rightBumper().getAsBoolean())
                .andThen(
                    Commands.parallel(
                            elevator.climbRetractAndLock().asProxy(), leds.setRainbowCmd())
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));
    // Heading reset
    controller
        .leftStick()
        .and(controller.rightStick())
        .onTrue(Commands.runOnce(() -> swerve.setYaw(new Rotation2d())));

    operator.leftTrigger().onTrue(Commands.runOnce(() -> currentTarget = Target.SPEAKER));
    operator
        .leftBumper()
        .onTrue(
            Commands.waitUntil(
                    () ->
                        Math.abs(
                                shooter
                                    .getAngle()
                                    .minus(ShooterSubsystem.PIVOT_MIN_ANGLE)
                                    .getDegrees())
                            < 10.0)
                .andThen(Commands.runOnce(() -> currentTarget = Target.AMP)));

    operator
        .a()
        .and(controller.rightTrigger().negate())
        .whileTrue(
            Commands.repeatingSequence(
                    shooter
                        .runFlywheelsCmd(() -> 0.0, () -> 0.0)
                        .until(
                            () ->
                                feeder.getFirstBeambreak() && swerve.getDistanceToSpeaker() < 8.0),
                    shooter
                        .runStateCmd(
                            () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRotation(),
                            () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getLeftRPS(),
                            () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRightRPS())
                        .until(() -> !feeder.getFirstBeambreak()))
                .until(controller.rightTrigger())
                .unless(controller.rightTrigger()));

    operator.start().whileTrue(elevator.runCurrentZeroing());
    operator
        .back()
        .onTrue(
            shooter
                .resetPivotPosition(ShooterSubsystem.PIVOT_MIN_ANGLE)
                .alongWith(
                    leds.setBlinkingCmd(new Color("#00ff00"), new Color(), 25.0)
                        .withTimeout(0.25)));

    autoChooser.addOption("None", Commands.none());
    autoChooser.addOption("Shoot Preload", teleopAutoAim());
    autoChooser.addDefaultOption("Amp 4 Wing", autoAmp4Wing());
    autoChooser.addOption("Source 3", autoSource3());
    autoChooser.addOption("Amp 5", autoAmp5());
    autoChooser.addOption("Source 4", autoSource4());
    autoChooser.addOption("Line Test Repeatedly", lineTest());

    // Dashboard command buttons
    SmartDashboard.putData("Shooter shoot", shootWithDashboard());
    SmartDashboard.putData("Run Swerve Azimuth Sysid", swerve.runModuleSteerCharacterizationCmd());
    SmartDashboard.putData("Run Swerve Drive Sysid", swerve.runDriveCharacterizationCmd());
    SmartDashboard.putData("Run Elevator Sysid", elevator.runSysidCmd());
    SmartDashboard.putData("Run Pivot Sysid", shooter.runPivotSysidCmd());
    SmartDashboard.putData("Run Flywheel Sysid", shooter.runFlywheelSysidCmd());
    SmartDashboard.putData(
        "manual zero shooter",
        shooter.resetPivotPosition(ShooterSubsystem.PIVOT_MIN_ANGLE).ignoringDisable(true));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // Update ascope mechanism visualization
    Logger.recordOutput(
        "Mechanism Poses",
        new Pose3d[] {
          shooter.getMechanismPose(), elevator.getCarriagePose(), elevator.getFirstStagePose()
        });
    Logger.recordOutput("Target", currentTarget);
    Logger.recordOutput("AutoAim/Speaker", FieldConstants.getSpeaker());
    // Logger.recordOutput("Canivore Util", CANBus.getStatus("canivore").BusUtilization);
    Logger.recordOutput(
        "Angle to target",
        Math.atan2(
            FieldConstants.getSpeaker().getY() - swerve.getPose().getY(),
            FieldConstants.getSpeaker().getX() - swerve.getPose().getX()));
    Logger.recordOutput(
        "AutoAim/Actual Distance",
        swerve.getPose().minus(FieldConstants.getSpeaker()).getTranslation().getNorm());
  }

  private Command shootWithDashboard() {
    return Commands.parallel(
        shooter.runStateCmd(
            () -> Rotation2d.fromDegrees(dashShotDegrees.get()),
            () -> dashShotLeftRPS.get(),
            () -> dashShotRightRPS.get()),
        feeder.indexCmd());
  }

  /**
   * A demo command that goes through all the steps of a shoot while moving algorithm
   *
   * @param speeds
   * @return A command that takes the robot through an auto aim sequence
   */
  private Command teleopAutoAim() {
    return Commands.sequence(
        swerve
            .runVelocityFieldRelative(
                () -> {
                  double velocity =
                      Math.sqrt(
                          Math.pow(swerve.getVelocity().vxMetersPerSecond, 2)
                              + Math.pow(swerve.getVelocity().vyMetersPerSecond, 2));
                  Rotation2d direction =
                      new Rotation2d(
                          swerve.getVelocity().vxMetersPerSecond,
                          swerve.getVelocity().vyMetersPerSecond);
                  double clampedSpeed =
                      MathUtil.clamp(
                          velocity,
                          -SwerveSubsystem.MAX_AUTOAIM_SPEED * 0.95,
                          SwerveSubsystem.MAX_AUTOAIM_SPEED * 0.95);
                  return new ChassisSpeeds(
                      clampedSpeed * direction.getCos(), clampedSpeed * direction.getSin(), 0.0);
                })
            .until(
                () ->
                    Math.sqrt(
                            Math.pow(swerve.getVelocity().vxMetersPerSecond, 2)
                                + Math.pow(swerve.getVelocity().vyMetersPerSecond, 2))
                        < SwerveSubsystem.MAX_AUTOAIM_SPEED),
        Commands.runOnce(
            () -> {
              AutoAimStates.curShotSpeeds =
                  new ChassisSpeeds(
                      swerve.getVelocity().vxMetersPerSecond,
                      swerve.getVelocity().vyMetersPerSecond,
                      0.0);
              Logger.recordOutput("AutoAim/Current Shot Speed", AutoAimStates.curShotSpeeds);
              AutoAimStates.lookaheadTime = swerve.getLookaheadTime();
              double distance =
                  swerve
                      .getLinearFuturePose(AutoAimStates.lookaheadTime, AutoAimStates.curShotSpeeds)
                      .minus(FieldConstants.getSpeaker())
                      .getTranslation()
                      .getNorm();
              AutoAimStates.curShotData = AutoAim.shotMap.get(distance);
              Logger.recordOutput("AutoAim/Distance From Target", distance);
              Logger.recordOutput(
                  "AutoAim/Desired Shooting Angle", AutoAim.shotMap.get(distance).getRotation());
              Logger.recordOutput(
                  "AutoAim/Actual Shooting Angle", AutoAimStates.curShotData.getRotation());
              AutoAimStates.endingPose =
                  swerve.getLinearFuturePose(
                      AutoAimStates.lookaheadTime, AutoAimStates.curShotSpeeds);
              AutoAimStates.virtualTarget =
                  AutoAim.getVirtualTarget(AutoAimStates.endingPose, AutoAimStates.curShotSpeeds);
              Logger.recordOutput("AutoAim/Virtual Target", AutoAimStates.virtualTarget);
              System.out.println(Timer.getFPGATimestamp());
            }),
        Commands.deadline(
            Commands.waitSeconds(AutoAimStates.lookaheadTime),
            swerve.teleopAimAtVirtualTargetCmd(
                () -> AutoAimStates.curShotSpeeds.vxMetersPerSecond,
                () -> AutoAimStates.curShotSpeeds.vyMetersPerSecond,
                AutoAimStates.lookaheadTime),
            shooter.runStateCmd(
                () -> AutoAimStates.curShotData.getRotation(),
                () -> AutoAimStates.curShotData.getLeftRPS(),
                () -> AutoAimStates.curShotData.getRightRPS())),
        // keeps moving to prevent the robot from stopping and changing the velocity of the note
        Commands.either(
            swerve
                .runVelocityFieldRelative(
                    () ->
                        new ChassisSpeeds(
                            AutoAimStates.curShotSpeeds.vxMetersPerSecond,
                            AutoAimStates.curShotSpeeds.vyMetersPerSecond,
                            0))
                .alongWith(feeder.runVelocityCmd(FeederSubsystem.INDEXING_VELOCITY))
                .withTimeout(0.25),
            staticAutoAim(),
            () ->
                shooter.isAtGoal()
                    && MathUtil.isNear(
                        AutoAimStates.endingPose
                            .getTranslation()
                            .minus(AutoAimStates.virtualTarget.getTranslation())
                            .getAngle()
                            .getDegrees(),
                        swerve.getRotation().getDegrees(),
                        2.0)));
  }

  private Command staticAutoAim(double rotationTolerance) {
    var headingController =
        new ProfiledPIDController(
            3.0,
            0.0,
            0.0,
            new Constraints(
                SwerveSubsystem.MAX_ANGULAR_SPEED * 0.75,
                SwerveSubsystem.MAX_ANGULAR_ACCELERATION * 0.75));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.deadline(
            feeder
                .runVelocityCmd(0.0)
                .until(
                    () ->
                        shooter.isAtGoal()
                            && MathUtil.isNear(
                                swerve
                                    .getPose()
                                    .getTranslation()
                                    .minus(FieldConstants.getSpeaker().getTranslation())
                                    .getAngle()
                                    .getDegrees(),
                                swerve.getPose().getRotation().getDegrees(),
                                rotationTolerance))
                .andThen(
                    feeder
                        .runVelocityCmd(FeederSubsystem.INDEXING_VELOCITY)
                        .raceWith(
                            Commands.waitUntil(() -> !feeder.getFirstBeambreak())
                                .andThen(Commands.waitSeconds(0.25)))),
            swerve.runVelocityFieldRelative(
                () -> {
                  var pidOut =
                      headingController.calculate(
                          swerve.getPose().getRotation().getRadians(),
                          swerve
                              .getPose()
                              .getTranslation()
                              .minus(FieldConstants.getSpeaker().getTranslation())
                              .getAngle()
                              .getRadians());
                  Logger.recordOutput(
                      "AutoAim/Static Heading Setpoint", headingController.getSetpoint().position);
                  Logger.recordOutput(
                      "AutoAim/Static Heading Setpoint Vel",
                      headingController.getSetpoint().velocity);
                  return new ChassisSpeeds(
                      0.0, 0.0, pidOut + (headingController.getSetpoint().velocity * 0.9));
                }),
            shooter.runStateCmd(
                () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRotation(),
                () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getLeftRPS(),
                () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRightRPS()))
        .beforeStarting(
            () ->
                headingController.reset(
                    new State(
                        swerve.getPose().getRotation().getRadians(),
                        swerve.getVelocity().omegaRadiansPerSecond)));
  }

  private Command staticAutoAim() {
    return staticAutoAim(swerve.getDistanceToSpeaker() < 2.5 ? 6.0 : 3.0);
  }

  private Command autoStaticAutoAim() {
    return feeder
        .indexCmd()
        .deadlineWith(
            swerve.runVelocityCmd(() -> new ChassisSpeeds()),
            Commands.repeatingSequence(
                shooter
                    .runFlywheelsCmd(() -> 0.0, () -> 0.0)
                    .unless(() -> feeder.getFirstBeambreak() && swerve.getDistanceToSpeaker() < 8.0)
                    .until(() -> feeder.getFirstBeambreak() && swerve.getDistanceToSpeaker() < 8.0),
                shooter.runStateCmd(
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRotation(),
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getLeftRPS(),
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRightRPS())))
        .until(() -> feeder.getFirstBeambreak())
        .unless(() -> feeder.getFirstBeambreak())
        .withTimeout(1.0)
        .andThen(
            staticAutoAim(6.0)
                .deadlineWith(
                    intake.runVoltageCmd(0, 0).asProxy(), carriage.runVoltageCmd(0).asProxy())
                .withTimeout(2.0)
                .unless(() -> !feeder.getFirstBeambreak()))
        .asProxy()
        .withTimeout(4.0);
  }

  private Command autoFenderShot() {
    return shooter
        .runStateCmd(
            AutoAim.FENDER_SHOT.getRotation(),
            AutoAim.FENDER_SHOT.getLeftRPS(),
            AutoAim.FENDER_SHOT.getRightRPS())
        .raceWith(
            feeder
                .runVelocityCmd(0.0)
                .until(() -> shooter.isAtGoal())
                .andThen(
                    feeder
                        .runVelocityCmd(FeederSubsystem.INDEXING_VELOCITY)
                        .raceWith(
                            Commands.waitUntil(() -> !feeder.getFirstBeambreak())
                                .andThen(Commands.waitSeconds(0.1)))))
        .asProxy();
  }

  private Command autoIntake() {
    return Commands.parallel(
        intake
            .runVelocityCmd(50.0, 30.0)
            .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
            .asProxy(),
        feeder.indexCmd().asProxy(),
        carriage
            .runVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE)
            .until(() -> feeder.getFirstBeambreak())
            .asProxy(),
        shooter
            .runStateCmd(
                () ->
                    Rotation2d.fromDegrees(
                        MathUtil.clamp(
                            AutoAim.shotMap
                                .get(swerve.getDistanceToSpeaker())
                                .getRotation()
                                .getDegrees(),
                            0,
                            35)),
                () -> 0.0,
                () -> 0.0)
            .until(() -> feeder.getFirstBeambreak())
            .unless(() -> feeder.getFirstBeambreak())
            .andThen(
                shooter.runStateCmd(
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRotation(),
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getLeftRPS(),
                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRightRPS()))
            .asProxy());
  }

  private Command autoAmp4Wing() {
    return Commands.sequence(
        autoFenderShot(),
        swerve
            .runChoreoTraj(Choreo.getTrajectory("amp 4 local.1"), true)
            .asProxy()
            .deadlineWith(autoIntake()),
        autoIntake()
            .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
            .withTimeout(1.0),
        autoStaticAutoAim()
            .andThen(Commands.print("Done with auto static auto aim"))
            .beforeStarting(Commands.print("Before auto static auto aim!!!")),
        swerve
            .runChoreoTraj(Choreo.getTrajectory("amp 4 local.2"))
            .asProxy()
            .deadlineWith(autoIntake()),
        autoIntake()
            .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
            .withTimeout(1.0),
        autoStaticAutoAim(),
        swerve
            .runChoreoTraj(Choreo.getTrajectory("amp 4 local.3"))
            .asProxy()
            .deadlineWith(autoIntake()),
        autoIntake().until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak()),
        autoStaticAutoAim());
  }

  private Command autoAmp5() {
    return Commands.sequence(
        autoFenderShot(),
        swerve
            .runChoreoTraj(Choreo.getTrajectory("amp 5.1"), true)
            .asProxy()
            .deadlineWith(autoIntake()),
        autoStaticAutoAim(),
        swerve
            .runChoreoTraj(Choreo.getTrajectory("amp 5.2"))
            .asProxy()
            .deadlineWith(Commands.waitSeconds(1.0).andThen(autoIntake())),
        autoStaticAutoAim(),
        swerve.runChoreoTraj(Choreo.getTrajectory("amp 5.3")).asProxy().deadlineWith(autoIntake()),
        autoIntake()
            .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
            .withTimeout(1.0),
        autoStaticAutoAim(),
        swerve.runChoreoTraj(Choreo.getTrajectory("amp 5.4")).asProxy().deadlineWith(autoIntake()),
        autoIntake()
            .until(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
            .withTimeout(1.0),
        autoStaticAutoAim());
  }

  private Command autoSource3() {
    return Commands.sequence(
        autoFenderShot(),
        swerve
            .runChoreoTraj(Choreo.getTrajectory("source 3.1"), true)
            .asProxy()
            .deadlineWith(autoIntake()),
        autoIntake()
            .raceWith(
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
                )
                )
            .withTimeout(1.0),
        autoStaticAutoAim().unless(() -> !feeder.getFirstBeambreak()),
        swerve
            .runChoreoTraj(Choreo.getTrajectory("source 3.2"))
            .asProxy()
            .deadlineWith(autoIntake()),
        autoIntake()
            .raceWith(
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
                )
                )
            .withTimeout(1.0),
        autoStaticAutoAim());
  }

  private Command autoSource4() {
    return Commands.sequence(
        autoFenderShot(),
        swerve.runChoreoTraj(Choreo.getTrajectory("source 4.1")).deadlineWith(autoIntake()),
        autoStaticAutoAim(),
        swerve.runChoreoTraj(Choreo.getTrajectory("source 4.2")).deadlineWith(autoIntake()),
        autoStaticAutoAim(),
        swerve.runChoreoTraj(Choreo.getTrajectory("source 4.3")).deadlineWith(autoIntake()),
        autoStaticAutoAim());
  }

  private Command lineTest() {
    ChoreoTrajectory traj = Choreo.getTrajectory("line test");
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              if (DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get().equals(Alliance.Red)) {
                swerve.setPose(traj.flipped().getInitialPose());
              } else {
                swerve.setPose(traj.getInitialPose());
              }
            }),
        swerve.runChoreoTraj(traj).repeatedly());
  }

  private Command ampHeadingSnap(DoubleSupplier x, DoubleSupplier y) {
    var headingController =
        new ProfiledPIDController(
            3.0,
            0.0,
            0.0,
            new Constraints(
                SwerveSubsystem.MAX_ANGULAR_SPEED * 0.75, SwerveSubsystem.MAX_ANGULAR_SPEED * 0.5));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    return swerve.runVoltageTeleopFieldRelative(
        () -> {
          double pidOut =
              headingController.calculate(swerve.getRotation().getRadians(), Math.PI / 2);
          return new ChassisSpeeds(
              x.getAsDouble(), y.getAsDouble(), pidOut + headingController.getSetpoint().velocity);
        });
  }

  private Command speakerHeadingSnap(DoubleSupplier x, DoubleSupplier y) {
    var headingController =
        new ProfiledPIDController(
            4.0,
            0.0,
            0.0,
            new Constraints(
                SwerveSubsystem.MAX_ANGULAR_SPEED * 0.75, SwerveSubsystem.MAX_ANGULAR_SPEED * 0.5));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    return swerve.runVoltageTeleopFieldRelative(
        () -> {
          double pidOut =
              headingController.calculate(
                  swerve.getRotation().getRadians(),
                  swerve
                      .getPose()
                      .getTranslation()
                      .minus(FieldConstants.getSpeaker().getTranslation())
                      .getAngle()
                      .getRadians());
          return new ChassisSpeeds(
              x.getAsDouble(),
              y.getAsDouble(),
              pidOut + (headingController.getSetpoint().velocity));
        });
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = autoChooser.get();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Modifies the given joystick axis value to make teleop driving smoother. */
  private static double teleopAxisAdjustment(double x) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(x, 2)) * Math.signum(x), 0.02);
  }
}
