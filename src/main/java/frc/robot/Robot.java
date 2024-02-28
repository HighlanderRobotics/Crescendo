// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.shooter.ShooterSubystem;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.AutoAimStates;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.dynamicauto.DynamicAuto;
import frc.robot.utils.dynamicauto.LocalADStarAK;
import frc.robot.utils.dynamicauto.Note;
import java.util.Map;
import java.util.function.Supplier;
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
    END
  }

  public static final RobotMode mode = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;
  public static final boolean USE_AUTO_AIM = true;
  private Command autonomousCommand;

  private final CommandXboxControllerSubsystem controller = new CommandXboxControllerSubsystem(0);
  private final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  private Target currentTarget = Target.SPEAKER;
  private double flywheelIdleSpeed = -0.1;

  private int dynamicAutoCounter = 0;
  private boolean atShootingLocation = false;
  private ChoreoTrajectory curTrajectory = Choreo.getTrajectory("Amp Side To C1");


  
  private double distance = 0;
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
                public void updateInputs(GyroIOInputs inputs) {}

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
  private final ShooterSubystem shooter =
      new ShooterSubystem(mode == RobotMode.REAL ? new ShooterIOReal() : new ShooterIOSim());
  private final CarriageSubsystem carriage = new CarriageSubsystem(new CarriageIOReal());
  // private final ReactionBarReleaseSubsystem reactionBarRelease =
  //     new ReactionBarReleaseSubsystem(new ReactionBarReleaseIOReal());
  private final LEDSubsystem leds =
      new LEDSubsystem(mode == RobotMode.REAL ? new LEDIOReal() : new LEDIOSim());

  private AutoManager autoManager = new AutoManager(swerve, intake, elevator, shooter, feeder);
  ;

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

    for (Note note : DynamicAuto.notes) {
      noteDropdown.addOption(note.getName(), note);
    }

    // Default Commands here
    swerve.setDefaultCommand(
        swerve.runVelocityFieldRelative(
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
    shooter.setDefaultCommand(
        shooter.runFlywheelsCmd(() -> flywheelIdleSpeed, () -> flywheelIdleSpeed));
    // reactionBarRelease.setDefaultCommand(
    //     reactionBarRelease.setRotationCmd(Rotation2d.fromDegrees(0.0)));
    leds.setDefaultCommand(
        leds.defaultStateDisplayCmd(
            () -> DriverStation.isEnabled(), () -> currentTarget == Target.SPEAKER));
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
    controller
        .leftTrigger()
        // .and(() -> !(carriage.getBeambreak() || feeder.getFirstBeambreak()))
        .whileTrue(intake.runVelocityCmd(80.0, 30.0));
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
        .whileTrue(
            Commands.parallel(
                teleopAutoAim(
                    () -> {
                      double vx = swerve.getVelocity().vxMetersPerSecond;
                      double vy = swerve.getVelocity().vyMetersPerSecond;
                      double omega = swerve.getVelocity().omegaRadiansPerSecond;

                      double polarVelocity =
                          MathUtil.clamp(
                              Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2)),
                              -SwerveSubsystem.MAX_LINEAR_SPEED / 5,
                              SwerveSubsystem.MAX_LINEAR_SPEED / 5);
                      Logger.recordOutput("AutoAim/Polar Velocity", polarVelocity);
                      double polarRadians = Math.atan2(vy, vx);
                      ChassisSpeeds polarSpeeds =
                          new ChassisSpeeds(
                              polarVelocity * Math.cos(polarRadians),
                              polarVelocity * Math.sin(polarRadians),
                              omega);
                      Logger.recordOutput("AutoAim/Polar Speeds", polarSpeeds);
                      return polarSpeeds;
                    }),
                Commands.waitSeconds(AutoAim.LOOKAHEAD_TIME_SECONDS)
                    .andThen(
                        Commands.parallel(
                            feeder.runVelocityCmd(FeederSubsystem.INDEXING_VELOCITY),
                            Commands.runOnce(
                                () ->
                                    Logger.recordOutput(
                                        "AutoAim/Shooting Pose", swerve.getPose()))))));
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
    // Heading reset
    controller
        .leftStick()
        .and(controller.rightStick())
        .onTrue(Commands.runOnce(() -> swerve.setYaw(new Rotation2d())));
    controller
        .a()
        .whileTrue(
            swerve.teleopPointTowardsTranslationCmd(
                () -> -controller.getLeftY() * SwerveSubsystem.MAX_LINEAR_SPEED,
                () -> -controller.getLeftX() * SwerveSubsystem.MAX_LINEAR_SPEED));

    // Test binding for elevator
    controller.b().whileTrue(elevator.setExtensionCmd(() -> 1.0));

    controller
        .x()
        .toggleOnTrue(swerve.runChoreoTraj(() -> DynamicAuto.makeNoteToNote(swerve::getPose)));
    controller
        .leftBumper()
        .toggleOnTrue(
            Commands.runOnce(
                () -> DynamicAuto.getAbsoluteClosestNote(swerve::getPose).blacklist(), swerve));
    controller
        .rightBumper()
        .toggleOnTrue(
            Commands.runOnce(
                () -> DynamicAuto.getAbsoluteClosestNote(swerve::getPose).whitelist(), swerve));

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

    NamedCommands.registerCommand(
        "auto aim amp 4 local sgmt 1", autonomousAutoAim("amp 4 local sgmt 1"));

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
    Logger.recordOutput("DynamicAuto/Current Path Ending Pose", curTrajectory.getFinalPose());
    Logger.recordOutput("DynamicAuto/Current Path Initial Pose", curTrajectory.getInitialPose());
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
    Logger.recordOutput("DynamicAuto/Curent Trajectory Followed", DynamicAuto.curTrajectory.getPoses());
    Logger.recordOutput("DynamicAuto/Forward Trajectory Followed", DynamicAuto.forwardLookingTrajectory);
    Logger.recordOutput("DynamicAuto/Whitelist Count", DynamicAuto.whitelistCount);
    // Logger.recordOutput("Canivore Util", CANBus.getStatus("canivore").BusUtilization);
  }

  public Command shootWithDashboard() {
    return Commands.parallel(
        shooter.runStateCmd(
            () -> Rotation2d.fromDegrees(degrees.get()), () -> leftRPS.get(), () -> rightRPS.get()),
        feeder.indexCmd());
  }

  public Command drivePath() {
    return swerve
        .runVelocityFieldRelative(
            () -> {
              System.out.println(AutoAimStates.elapsedAutonomousSeconds);
              AutoAimStates.curState = swerve.getAutoState(AutoAimStates.elapsedAutonomousSeconds);
              AutoAimStates.elapsedAutonomousSeconds +=
                  Timer.getFPGATimestamp()
                      - AutoAimStates.elapsedAutonomousSeconds
                      - AutoAimStates.startingAutonomousSeconds;
              return new ChassisSpeeds(
                  AutoAimStates.curState.velocityX,
                  AutoAimStates.curState.velocityY,
                  AutoAimStates.curState.angularVelocity);
            })
        .until(
            () -> {
              return AutoAimStates.elapsedAutonomousSeconds >= AutoAimStates.pathTotalTime;
            })
        .andThen(
            () -> {
              AutoAimStates.startingAutonomousSeconds = Timer.getFPGATimestamp();
            },
            swerve);
  }

  /**
   * A demo command that goes through all the steps of a shoot while moving algorithm Has print
   * commands for any unimplemented functionality
   *
   * @param speeds
   * @return A command that takes the robot through an auto aim sequence
   */
  public Command teleopAutoAim(Supplier<ChassisSpeeds> speeds) {
    Command getInitialValues =
        Commands.runOnce(
            () -> {
              AutoAimStates.curShotSpeeds = speeds.get();
              Logger.recordOutput("AutoAim/cur shot speedd", AutoAimStates.curShotSpeeds);
              double distance =
                  swerve
                      .getLinearFuturePose(
                          AutoAim.LOOKAHEAD_TIME_SECONDS, AutoAimStates.curShotSpeeds)
                      .minus(FieldConstants.getSpeaker())
                      .getTranslation()
                      .getNorm();
              AutoAimStates.curShotData = AutoAim.shotMap.get(distance);
              Logger.recordOutput("AutoAim/Distance From Target", distance);
              Logger.recordOutput(
                  "AutoAim/Desired Shooting Angle", AutoAim.shotMap.get(distance).getRotation());
              Logger.recordOutput(
                  "AutoAim/Actual Shooting Angle", AutoAimStates.curShotData.getRotation());
              System.out.println(Timer.getFPGATimestamp());
            });
    Command runRobot =
        Commands.parallel(
            shooter.runStateCmd(
                () -> AutoAimStates.curShotData.getRotation(),
                () -> AutoAimStates.curShotData.getLeftRPS(),
                () -> AutoAimStates.curShotData.getRightRPS()),
            swerve.teleopPointTowardsTranslationCmd(
                () -> AutoAimStates.curShotSpeeds.vxMetersPerSecond,
                () -> AutoAimStates.curShotSpeeds.vyMetersPerSecond,
                AutoAim.LOOKAHEAD_TIME_SECONDS));
    return Commands.sequence(
        getInitialValues,
        Commands.deadline(
            Commands.waitSeconds(AutoAim.LOOKAHEAD_TIME_SECONDS),
            runRobot,
            Commands.print(String.valueOf(AutoAimStates.curShotSpeeds.vxMetersPerSecond))),
        // keeps moving to prevent the robot from stopping and changing the velocity of the note
        swerve
            .runVelocityFieldRelative(
                () ->
                    new ChassisSpeeds(
                        AutoAimStates.curShotSpeeds.vxMetersPerSecond,
                        AutoAimStates.curShotSpeeds.vyMetersPerSecond,
                        0))
            .withTimeout(0));
  }

  public Command autonomousAutoAim(String pathName) {

    return Commands.sequence(
        Commands.deadline(
                Commands.waitSeconds(AutoAim.LOOKAHEAD_TIME_SECONDS),
                Commands.parallel(
                    shooter.runStateCmd(
                        AutoAimStates.curShotData::getRotation,
                        AutoAimStates.curShotData::getLeftRPS,
                        AutoAimStates.curShotData::getRightRPS),
                    swerve.autonomousPointTowardsTranslationCmd()))
            .beforeStarting(
                () -> {
                  AutoAimStates.pathName = pathName;
                  AutoAimStates.pathTotalTime = Choreo.getTrajectory(pathName).getTotalTime();
                },
                swerve),
        Commands.print("Whoosh!"),
        drivePath());
    // keeps moving to prevent the robot from stopping and changing the velocity of the note

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
      return AutoStepSelector.DYNAMIC_TO_NOTE;
    } else {
      atShootingLocation = true;
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
                        Commands.race(
                            DynamicAuto.noteToShoot(swerve),
                            shooter.runStateCmd(
                                () -> AutoAim.shotMap.get(distance).getRotation(),
                                () -> AutoAim.shotMap.get(distance).getLeftRPS(),
                                () -> AutoAim.shotMap.get(distance).getRightRPS()))),
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
                                () -> AutoAim.shotMap.get(distance).getRotation(),
                                () -> AutoAim.shotMap.get(distance).getLeftRPS(),
                                () -> AutoAim.shotMap.get(distance).getRightRPS())))),
                this::selectAuto),
            Commands.runOnce(
                () -> {
                  dynamicAutoCounter++;
                  System.out.println(dynamicAutoCounter);
                  distance =
                      curTrajectory
                          .getFinalPose()
                          .minus(FieldConstants.getSpeaker())
                          .getTranslation()
                          .getNorm();
                }))
        .beforeStarting(
            () -> {swerve.setPose(DynamicAuto.startingLocations[1].getPoseAllianceSpecific()); DynamicAuto.updateWhitelistCount();})
        .asProxy();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    dynamicAutoCounter = 0;
    dynamicAuto().schedule();
  }

  @Override
  public void teleopInit() {
    dynamicAuto().cancel();
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
