// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.carriage.CarriageIOReal;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberSubsystem;
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
import frc.robot.utils.dynamicauto.DynamicAuto;
import frc.robot.utils.dynamicauto.LocalADStarAK;
import frc.robot.utils.dynamicauto.Note;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
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

  private static enum AutoStepSelector {
    START_TO_NOTE,
    NOTE_TO_SHOOT,
    NOTE_TO_NOTE,
    SHOOT_TO_NOTE,
    DYNAMIC_TO_NOTE,
    DYNAMIC_TO_SHOOT,
    SHOOT,
    END
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
  private double flywheelIdleSpeed = 0.0;

  private int dynamicAutoCounter = 0;
  private boolean atShootingLocation = false;
  private boolean justShotWNote = false;
  private ChoreoTrajectory curTrajectory = Choreo.getTrajectory("Amp Side To C1");

  private LoggedDashboardNumber degrees = new LoggedDashboardNumber("Rotation (degrees)", 37.0);
  private LoggedDashboardNumber leftRPS =
      new LoggedDashboardNumber("Left RPS (Rotations Per Sec)", 60.0);
  private LoggedDashboardNumber rightRPS =
      new LoggedDashboardNumber("Right RPS (Rotations Per Sec)", 80.0);
  private LoggedDashboardNumber priority = new LoggedDashboardNumber("Note Priority");
  private LoggedDashboardBoolean blacklist = new LoggedDashboardBoolean("Note Blacklisted");

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
  private final ClimberSubsystem climber = new ClimberSubsystem(new ClimberIOReal());
  private LoggedDashboardChooser<Note> noteDropdown =
      new LoggedDashboardChooser<Note>("Note Picker");

  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStarAK());
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

    for (Note note : DynamicAuto.notes) {
      noteDropdown.addOption(note.getName(), note);
    }
    DynamicAuto.updateWhitelistCount();

    // Default Commands here
    swerve.setDefaultCommand(
        swerve.runVelocityFieldRelativeTeleopCmd(
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
                    carriage.runVoltageCmd(0.5).until(() -> !feeder.getFirstBeambreak()))
                .until(() -> currentTarget != Target.SPEAKER)));
    intake.setDefaultCommand(intake.runVoltageCmd(0.0, 0.0));
    shooter.setDefaultCommand(
        shooter.runFlywheelsCmd(() -> flywheelIdleSpeed, () -> flywheelIdleSpeed));
    climber.setDefaultCommand(climber.runVoltageCmd(0.0));
    leds.setDefaultCommand(
        leds.defaultStateDisplayCmd(
            () -> DriverStation.isEnabled(), () -> currentTarget == Target.SPEAKER));

    controller.setDefaultCommand(controller.rumbleCmd(0.0, 0.0));
    operator.setDefaultCommand(operator.rumbleCmd(0.0, 0.0));

    // Robot state management bindings
    new Trigger(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
        .debounce(0.25)
        .whileTrue(
            Commands.parallel(
                    controller.rumbleCmd(1.0, 1.0),
                    leds.setBlinkingCmd(new Color("#ff4400"), new Color("#000000"), 25.0))
                .withTimeout(0.5));

    // ---- Controller bindings here ----
    controller.leftTrigger().whileTrue(intake.runVelocityCmd(60.0, 30.0));
    controller
        .rightTrigger()
        .and(() -> currentTarget == Target.SPEAKER)
        .whileTrue(
            Commands.parallel(
                shooter.runStateCmd(Rotation2d.fromDegrees(80.0), 50.0, 40.0),
                Commands.waitSeconds(0.5).andThen(feeder.runVelocityCmd(24.0)))); // TODO tune
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
        .whileTrue(elevator.setExtensionCmd(() -> ElevatorSubsystem.AMP_EXTENSION_METERS))
        .onFalse(
            Commands.parallel(
                    carriage.runVoltageCmd(-3.0),
                    elevator.setExtensionCmd(() -> ElevatorSubsystem.AMP_EXTENSION_METERS))
                .withTimeout(0.75));
    controller.rightBumper().whileTrue(swerve.stopWithXCmd());
    controller
        .x()
        .whileTrue(
            Commands.parallel(
                shooter.runFlywheelVoltageCmd(ShooterSubsystem.PIVOT_MIN_ANGLE, -5.0),
                feeder.runVoltageCmd(-5.0),
                carriage.runVoltageCmd(-5.0),
                intake.runVelocityCmd(-50.0, -50.0)));
    controller
        .y()
        .and(() -> climber.getRotations() > 0.9 * ClimberSubsystem.CLIMB_ROTATIONS)
        .onTrue(
            // Commands.sequence(
            climber
                .retractClimbCmd()
                .until(() -> climber.getRotations() < 0.1) // TODO find actual tolerances
                // Commands.waitUntil(() -> controller.y().getAsBoolean()),
                // Commands.parallel(
                //     carriage.runVoltageCmd(-CarriageSubsystem.INDEXING_VOLTAGE),
                //     elevator.setExtensionCmd(() -> ElevatorSubsystem.TRAP_EXTENSION_METERS))
                // )
                .alongWith(
                    leds.setRainbowCmd(),
                    shooter.runStateCmd(Rotation2d.fromDegrees(90.0), 0.0, 0.0)));

    // Prep climb
    operator
        .rightTrigger(0.5)
        .debounce(0.25)
        .toggleOnFalse(
            Commands.parallel(
                Commands.waitUntil(operator.rightBumper())
                    .andThen(
                        elevator.setExtensionCmd(() -> ElevatorSubsystem.TRAP_EXTENSION_METERS)),
                Commands.waitUntil(() -> shooter.getAngle().getDegrees() > 80.0)
                    .andThen(
                        climber.extendRotationsCmd(
                            () -> ClimberSubsystem.CLIMB_ROTATIONS + (-operator.getLeftY() * 2.0))),
                shooter.runStateCmd(Rotation2d.fromDegrees(90.0), 0.0, 0.0),
                leds.setBlinkingCmd(new Color("#ff0000"), new Color("#ffffff"), 10.0)
                    .until(() -> climber.getRotations() > ClimberSubsystem.CLIMB_ROTATIONS * 0.9)
                    .andThen(
                        leds.setBlinkingCmd(new Color("#777700"), new Color("#ffffff"), 10.0))));
    // Heading reset
    controller
        .leftStick()
        .and(controller.rightStick())
        .onTrue(Commands.runOnce(() -> swerve.setYaw(new Rotation2d())));

    operator.leftTrigger().onTrue(Commands.runOnce(() -> currentTarget = Target.SPEAKER));
    operator.leftBumper().onTrue(Commands.runOnce(() -> currentTarget = Target.AMP));
    operator.a().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = 0.0));
    operator.b().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = 20.0));
    operator.x().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = 20.0));
    operator.y().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = 80.0));

    operator
        .start()
        .whileTrue(
            shooter.resetPivotPosition(
                ShooterSubsystem
                    .PIVOT_MIN_ANGLE)) // removing current zeroing for now because backlash is a
        // thing
        .whileTrue(elevator.runCurrentZeroing());
    operator.back().whileTrue(climber.runClimberCurrentZeroing());
    NamedCommands.registerCommand("stop", swerve.stopWithXCmd().asProxy());
    NamedCommands.registerCommand("intake", intake.runVelocityCmd(60.0, 30.0).asProxy());
    NamedCommands.registerCommand(
        "shoot",
        feeder
            .indexCmd()
            .alongWith(carriage.runVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE))
            .until(() -> feeder.getFirstBeambreak())
            .withTimeout(2.0)
            .andThen(
                Commands.race(
                        feeder
                            .runVelocityCmd(0.0)
                            .until(() -> shooter.isAtGoal())
                            .andThen(
                                feeder
                                    .runVelocityCmd(FeederSubsystem.INDEXING_VELOCITY)
                                    .withTimeout(0.5)),
                        shooter.runStateCmd(
                            () ->
                                AutoAim.shotMap
                                    .get(
                                        swerve.getDistanceToSpeaker())
                                    .getRotation(),
                            () ->
                                AutoAim.shotMap
                                    .get(
                                        swerve.getDistanceToSpeaker())
                                    .getLeftRPS(),
                            () ->
                                AutoAim.shotMap
                                    .get(
                                        swerve.getDistanceToSpeaker())
                                    .getRightRPS()))
                    .unless(() -> !feeder.getFirstBeambreak()))
            .asProxy());

    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption("Shoot Preload", teleopAutoAim());
    autoChooser.addOption("Amp 4 Wing", new PathPlannerAuto("local 4"));
    autoChooser.addOption("Source 3", new PathPlannerAuto("source 3"));
    autoChooser.addOption("Dynamic", dynamicAuto());

    SmartDashboard.putData(
        "Whitelist Note",
        Commands.runOnce(
            () -> DynamicAuto.getAbsoluteClosestNote(swerve::getPose).whitelist(), swerve));
    SmartDashboard.putData(
        "Blacklist All",
        Commands.runOnce(
            () -> {
              for (Note note : DynamicAuto.notes) {
                note.blacklist();
              }
            },
            swerve));
    SmartDashboard.putData(
        "Whitelist All",
        Commands.runOnce(
            () -> {
              for (Note note : DynamicAuto.notes) {
                note.whitelist();
              }
            },
            swerve));

    // Dashboard command buttons
    SmartDashboard.putData("Shooter shoot", shootWithDashboard());
    SmartDashboard.putData("Run Swerve Azimuth Sysid", swerve.runModuleSteerCharacterizationCmd());
    SmartDashboard.putData("Run Swerve Drive Sysid", swerve.runDriveCharacterizationCmd());
    SmartDashboard.putData("Run Elevator Sysid", elevator.runSysidCmd());
    SmartDashboard.putData("Run Pivot Sysid", shooter.runPivotSysidCmd());
    SmartDashboard.putData("Run Flywheel Sysid", shooter.runFlywheelSysidCmd());
    SmartDashboard.putData("Update Note", updateNote());
    SmartDashboard.putData(
        "Set robot pose center",
        Commands.runOnce(
            () -> swerve.setPose(DynamicAuto.startingLocations[1].getPoseAllianceSpecific())));

    SmartDashboard.putData("Dynamic Auto", dynamicAuto());
    SmartDashboard.putData("Zero shooter", shooter.runPivotCurrentZeroing());
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
    Logger.recordOutput(
        "DynamicAuto/Closest Note Within Distance",
        swerve
                .getPose()
                .minus(DynamicAuto.getClosestNote(swerve::getPose).getPoseAllianceSpicific())
                .getTranslation()
                .getNorm()
            > 1.5);

    Logger.recordOutput(
        "DynamicAuto/Closest Note Exists",
        DynamicAuto.getClosestNote(swerve::getPose).getExistence());
    Logger.recordOutput(
        "DynamicAuto/Closest Note",
        DynamicAuto.getClosestNote(swerve::getPose).getPoseAllianceSpicific());

    Logger.recordOutput(
        "DynamicAuto/Absolute Closest Note",
        DynamicAuto.getAbsoluteClosestNote(swerve::getPose).getPoseAllianceSpicific());

    Logger.recordOutput(
        "DynamicAuto/Closest Shooting Location",
        DynamicAuto.closestShootingLocation(() -> swerve.getPose(), DynamicAuto.shootingLocations)
            .getPoseAllianceSpecific());
    if (DynamicAuto.curTrajectory.isPresent()) {
      Logger.recordOutput(
          "DynamicAuto/Current Path Ending Pose", DynamicAuto.curTrajectory.get().getFinalPose());
      Logger.recordOutput(
          "DynamicAuto/Current Path Initial Pose",
          DynamicAuto.curTrajectory.get().getInitialPose());
      Logger.recordOutput(
          "DynamicAuto/Curent Trajectory Followed", DynamicAuto.curTrajectory.get().getPoses());
    }
    Logger.recordOutput("DynamicAuto/Whitelist Count", DynamicAuto.whitelistCount);

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

  public Command staticAutoAim() {
    var headingController =
        new ProfiledPIDController(
            5.0,
            0.0,
            0.01,
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
            Commands.parallel(
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
                                    3.0))
                    .andThen(feeder.runVelocityCmd(FeederSubsystem.INDEXING_VELOCITY)),
                shooter.runStateCmd(
                    () ->
                        AutoAim.shotMap
                            .get(swerve.getDistanceToSpeaker())
                            .getRotation(),
                    () ->
                        AutoAim.shotMap
                            .get(swerve.getDistanceToSpeaker())
                            .getLeftRPS(),
                    () ->
                        AutoAim.shotMap
                            .get(swerve.getDistanceToSpeaker())
                            .getRightRPS())));
  }

  public Command updateNote() {
    return Commands.runOnce(
        () ->
            DynamicAuto.updateNote(
                noteDropdown.get(), () -> blacklist.get(), () -> (int) priority.get()));
  }

  public Command blacklistNote() {
    return Commands.runOnce(() -> DynamicAuto.getAbsoluteClosestNote(swerve::getPose).blacklist());
  }

  public AutoStepSelector selectAuto() {
    if (dynamicAutoCounter == 0) {

      System.out.println("start to note");
      return AutoStepSelector.START_TO_NOTE;
    } else if (DynamicAuto.whitelistCount == 0) {
      return AutoStepSelector.END;
    } else if (atShootingLocation) {
      atShootingLocation = false;
      System.out.println("shoot to note");

      return AutoStepSelector.SHOOT_TO_NOTE;
    } else if (!justShotWNote
        && (DynamicAuto.noteClosest.getName().equals("W1")
            || DynamicAuto.noteClosest.getName().equals("W2")
            || DynamicAuto.noteClosest.getName().equals("W3"))) {
      System.out.println("Shoot note");
      justShotWNote = true;
      return AutoStepSelector.SHOOT;
    } else if ((carriage.getBeambreak() || feeder.getFirstBeambreak())
        || (DynamicAuto.getAbsoluteClosestNote(swerve::getPose).getExistence()
            && swerve
                    .getPose()
                    .minus(
                        DynamicAuto.getAbsoluteClosestNote(swerve::getPose)
                            .getPoseAllianceSpicific())
                    .getTranslation()
                    .getNorm()
                < 1.5
            && mode == RobotMode.SIM)) {
      System.out.println("note to shoot");
      atShootingLocation = true;
      justShotWNote = false;
      System.out.println(atShootingLocation);
      DynamicAuto.getAbsoluteClosestNote(swerve::getPose).blacklist();
      return AutoStepSelector.NOTE_TO_SHOOT;
    } else if ((!(carriage.getBeambreak() || feeder.getFirstBeambreak()) && Robot.isReal())
        || (!DynamicAuto.getAbsoluteClosestNote(swerve::getPose).getExistence()
            && swerve
                    .getPose()
                    .minus(
                        DynamicAuto.getAbsoluteClosestNote(swerve::getPose)
                            .getPoseAllianceSpicific())
                    .getTranslation()
                    .getNorm()
                < 1.5
            && mode == RobotMode.SIM)) {

      System.out.println("note to note 1");
      justShotWNote = false;
      DynamicAuto.getAbsoluteClosestNote(swerve::getPose).blacklist();
      return AutoStepSelector.NOTE_TO_NOTE;
    } else if ((carriage.getBeambreak() || feeder.getFirstBeambreak())
        || (DynamicAuto.getAbsoluteClosestNote(swerve::getPose).getExistence()
            && swerve
                    .getPose()
                    .minus(
                        DynamicAuto.getAbsoluteClosestNote(swerve::getPose)
                            .getPoseAllianceSpicific())
                    .getTranslation()
                    .getNorm()
                > 1.5)) {

      System.out.println("dynamic to note");
      justShotWNote = false;
      return AutoStepSelector.DYNAMIC_TO_NOTE;
    }
    if (DynamicAuto.whitelistCount <= 0) {
      System.out.println("END");
      return AutoStepSelector.END;
    } else {
      atShootingLocation = true;
      justShotWNote = false;
      System.out.println("dynamoci to shoot");
      return AutoStepSelector.DYNAMIC_TO_SHOOT;
    }
  }

  public Command dynamicAuto() {
    return Commands.repeatingSequence(
            Commands.select(
                Map.ofEntries(
                    Map.entry(
                        AutoStepSelector.NOTE_TO_NOTE,
                        Commands.race(
                            DynamicAuto.noteToNote(swerve), intake.runVelocityCmd(80.0, 30.0))),
                    Map.entry(
                        AutoStepSelector.NOTE_TO_SHOOT,
                        Commands.sequence(
                            Commands.race(
                                DynamicAuto.noteToShoot(swerve),
                                shooter.runStateCmd(
                                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRotation(),
                                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getLeftRPS(),
                                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRightRPS()),
                                feeder.runVoltageCmd(FeederSubsystem.INDEXING_VOLTAGE)))),
                    Map.entry(
                        AutoStepSelector.START_TO_NOTE,
                        Commands.race(
                            DynamicAuto.startToNote(swerve), intake.runVelocityCmd(80.0, 30.0))),
                    Map.entry(
                        AutoStepSelector.SHOOT_TO_NOTE,
                        Commands.race(
                            DynamicAuto.shootToNote(swerve), intake.runVelocityCmd(80.0, 30.0))),
                    Map.entry(
                        AutoStepSelector.DYNAMIC_TO_NOTE,
                        Commands.race(
                            DynamicAuto.DynamicToNote(swerve::getPose),
                            intake.runVelocityCmd(80.0, 30.0))),
                    Map.entry(
                        AutoStepSelector.DYNAMIC_TO_SHOOT,
                        Commands.race(
                            DynamicAuto.DynamicToShoot(swerve::getPose),
                            shooter.runStateCmd(
                                () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRotation(),
                                () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getLeftRPS(),
                                () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRightRPS()))),
                    Map.entry(
                        AutoStepSelector.SHOOT,
                        Commands.parallel(
                                shooter.runStateCmd(
                                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRotation(),
                                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getLeftRPS(),
                                    () -> AutoAim.shotMap.get(swerve.getDistanceToSpeaker()).getRightRPS()),
                                feeder.indexCmd())
                            .withTimeout(0.75)),
                    Map.entry(AutoStepSelector.END, swerve.stopWithXCmd())),
                this::selectAuto),
            Commands.runOnce(
                () -> {
                  dynamicAutoCounter++;
                  DynamicAuto.updateWhitelistCount();
                  System.out.println(dynamicAutoCounter);
                  System.out.println(DynamicAuto.whitelistCount);
                }),
            Commands.race(Commands.waitSeconds(0.5), intake.runVelocityCmd(80, 30)))
        .beforeStarting(
            () -> {
              swerve.setPose(DynamicAuto.startingLocations[1].getPoseAllianceSpecific());
              DynamicAuto.updateWhitelistCount();
            })
        .asProxy();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    dynamicAutoCounter = 0;
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
