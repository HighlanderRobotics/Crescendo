// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
  private LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  private final CommandXboxControllerSubsystem controller = new CommandXboxControllerSubsystem(0);
  private final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

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
        // Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
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
        swerve.runVelocityTeleopFieldRelative(
            () ->
                new ChassisSpeeds(
                    -teleopAxisAdjustment(controller.getLeftY()) * 12.0,
                    -teleopAxisAdjustment(controller.getLeftX()) * 12.0,
                    -teleopAxisAdjustment(controller.getRightX()) * 12.0)));
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
    shooter.setDefaultCommand(
        shooter.runFlywheelsCmd(() -> flywheelIdleSpeed, () -> flywheelIdleSpeed));
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
                    leds.setBlinkingCmd(new Color("#ff4400"), new Color("#000000"), 25.0))
                .withTimeout(0.5));

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
        .and(() -> !USE_AUTO_AIM)
        .whileTrue(shootWithDashboard())
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
                shooter.runFlywheelVoltageCmd(Rotation2d.fromDegrees(60.0), -5.0),
                feeder.runVoltageCmd(-5.0),
                carriage.runVoltageCmd(-5.0),
                intake.runVelocityCmd(-50.0, -50.0)));

    // Prep climb
    operator
        .rightTrigger(0.5)
        .debounce(0.25)
        .toggleOnFalse(
            Commands.parallel(
                elevator.setExtensionCmd(() -> ElevatorSubsystem.CLIMB_EXTENSION_METERS),
                leds.setBlinkingCmd(new Color("#00ff00"), new Color("#ffffff"), 10.0)));
    // Heading reset
    controller
        .leftStick()
        .and(controller.rightStick())
        .onTrue(Commands.runOnce(() -> swerve.setYaw(new Rotation2d())));

    operator.leftTrigger().onTrue(Commands.runOnce(() -> currentTarget = Target.SPEAKER));
    operator.leftBumper().onTrue(Commands.runOnce(() -> currentTarget = Target.AMP));
    operator.a().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = -0.1));
    operator.b().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = 20.0));
    operator.x().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = 20.0));
    operator.y().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = 80.0));

    operator.start().whileTrue(elevator.runCurrentZeroing());
    operator
        .back()
        .onTrue(
            shooter
                .resetPivotPosition(ShooterSubsystem.PIVOT_MIN_ANGLE)
                .alongWith(
                    leds.setBlinkingCmd(new Color("#00ff00"), new Color(), 25.0)
                        .withTimeout(0.25)));
    NamedCommands.registerCommand("stop", swerve.stopWithXCmd());
    NamedCommands.registerCommand(
        "intake",
        Commands.parallel(
            intake.runVelocityCmd(90.0, 30.0),
            feeder.indexCmd(),
            carriage.runVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE),
            shooter.runStateCmd(ShooterSubsystem.PIVOT_MIN_ANGLE, 60, 80)));
    NamedCommands.registerCommand(
        "fender",
        shooter
            .runStateCmd(
                AutoAim.shotMap.get(1.5).getRotation(),
                AutoAim.shotMap.get(1.5).getLeftRPS(),
                AutoAim.shotMap.get(1.5).getRightRPS())
            .raceWith(
                feeder
                    .runVelocityCmd(0.0)
                    .until(() -> shooter.isAtGoal())
                    .andThen(
                        feeder
                            .runVelocityCmd(FeederSubsystem.INDEXING_VELOCITY)
                            .withTimeout(0.5))));
    NamedCommands.registerCommand(
        "shoot",
        feeder
            .indexCmd()
            .alongWith(carriage.runVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE))
            .until(() -> feeder.getFirstBeambreak())
            .withTimeout(1.0)
            .andThen(staticAutoAim(5.0).withTimeout(2.0)));

    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption("Shoot Preload", teleopAutoAim());
    autoChooser.addOption("Amp 4 Wing", new PathPlannerAuto("local 4"));
    autoChooser.addOption("Source 3", new PathPlannerAuto("source 3"));
    autoChooser.addOption("Amp 5", new PathPlannerAuto("amp 5"));
    autoChooser.addOption("Source 4", new PathPlannerAuto("source 4"));
    autoChooser.addOption("Line Test Repeatedly", new PathPlannerAuto("line test").repeatedly());

    // Dashboard command buttons
    SmartDashboard.putData("Shooter shoot", shootWithDashboard());
    SmartDashboard.putData("Run Swerve Azimuth Sysid", swerve.runModuleSteerCharacterizationCmd());
    SmartDashboard.putData("Run Swerve Drive Sysid", swerve.runDriveCharacterizationCmd());
    SmartDashboard.putData("Run Elevator Sysid", elevator.runSysidCmd());
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
        "AutoAIm/Actual Distance",
        swerve.getPose().minus(FieldConstants.getSpeaker()).getTranslation().getNorm());
  }

  private LoggedDashboardNumber degrees = new LoggedDashboardNumber("Rotation (degrees)", 37.0);
  private LoggedDashboardNumber leftRPS =
      new LoggedDashboardNumber("Left RPS (Rotations Per Sec)", 60.0);
  private LoggedDashboardNumber rightRPS =
      new LoggedDashboardNumber("Right RPS (Rotations Per Sec)", 80.0);

  public Command shootWithDashboard() {
    return Commands.parallel(
        shooter.runStateCmd(
            () -> Rotation2d.fromDegrees(degrees.get()), () -> leftRPS.get(), () -> rightRPS.get()),
        feeder.indexCmd());
  }

  /**
   * A demo command that goes through all the steps of a shoot while moving algorithm
   *
   * @param speeds
   * @return A command that takes the robot through an auto aim sequence
   */
  public Command teleopAutoAim() {
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

  public Command staticAutoAim(double rotationTolerance) {
    var headingController =
        new ProfiledPIDController(
            5.0,
            0.0,
            0.0,
            new Constraints(
                SwerveSubsystem.MAX_ANGULAR_SPEED * 0.75, SwerveSubsystem.MAX_ANGULAR_SPEED * 0.5));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.runOnce(
            () ->
                headingController.reset(
                    new State(
                        swerve.getPose().getRotation().getRadians(),
                        swerve.getVelocity().omegaRadiansPerSecond)))
        .andThen(
            Commands.deadline(
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
                                    rotationTolerance)
                                && MathUtil.isNear(
                                    0.0,
                                    swerve.getVelocity().omegaRadiansPerSecond,
                                    Units.degreesToRadians(90.0)))
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
                      return new ChassisSpeeds(
                          0.0, 0.0, pidOut + headingController.getSetpoint().velocity);
                    }),
                shooter.runStateCmd(
                    () ->
                        AutoAim.shotMap
                            .get(
                                swerve
                                    .getPose()
                                    .minus(FieldConstants.getSpeaker())
                                    .getTranslation()
                                    .getNorm())
                            .getRotation(),
                    () ->
                        AutoAim.shotMap
                            .get(
                                swerve
                                    .getPose()
                                    .minus(FieldConstants.getSpeaker())
                                    .getTranslation()
                                    .getNorm())
                            .getLeftRPS(),
                    () ->
                        AutoAim.shotMap
                            .get(
                                swerve
                                    .getPose()
                                    .minus(FieldConstants.getSpeaker())
                                    .getTranslation()
                                    .getNorm())
                            .getRightRPS())));
  }

  public Command staticAutoAim() {
    return staticAutoAim(3.0);
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
    return swerve.runVelocityTeleopFieldRelative(
        () -> {
          double pidOut =
              headingController.calculate(
                  swerve.getRotation().getRadians(),
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                      ? Math.PI / 2
                      : -Math.PI / 2);
          return new ChassisSpeeds(
              x.getAsDouble(), y.getAsDouble(), pidOut + headingController.getSetpoint().velocity);
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
