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

  private static enum CommandSelector {
    START_TO_NOTE,
    NOTE_TO_SHOOT,
    NOTE_TO_NOTE,
    SHOOT_TO_NOTE,
    DYNAMIC_TO_NOTE,
    DYNAMIC_TO_SHOOT
  }

  public static final RobotMode mode = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;
  private Command autonomousCommand;

  private final CommandXboxControllerSubsystem controller = new CommandXboxControllerSubsystem(0);
  private final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  private Target currentTarget = Target.SPEAKER;
  private double flywheelIdleSpeed = -0.1;

  private int dynamicAutoCounter = 0;
  private boolean atShootingLocation = false;
  private ChoreoTrajectory curTrajectory = Choreo.getTrajectory("Amp Side To C1");
  private Pose2d[] forwardLookingTrajectory = curTrajectory.getPoses();

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
            feeder.indexCmd().until(() -> currentTarget == Target.AMP),
            Commands.sequence(
                    feeder
                        .runVoltageCmd(-FeederSubsystem.INDEXING_VOLTAGE)
                        .until(() -> carriage.getBeambreak()),
                    feeder.runVoltageCmd(-FeederSubsystem.INDEXING_VOLTAGE).withTimeout(0.5),
                    feeder.runVoltageCmd(0.0))
                .until(() -> currentTarget == Target.SPEAKER)));
    carriage.setDefaultCommand(
        Commands.repeatingSequence(
            carriage.indexBackwardsCmd().until(() -> currentTarget == Target.AMP),
            Commands.sequence(
                    carriage
                        .runVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE)
                        .until(() -> feeder.getFirstBeambreak()),
                    carriage.runVoltageCmd(CarriageSubsystem.INDEXING_VOLTAGE).withTimeout(0.5),
                    carriage.runVoltageCmd(0.0))
                .until(() -> currentTarget == Target.SPEAKER)));
    intake.setDefaultCommand(intake.runVoltageCmd(0.0, 0.0));
    shooter.setDefaultCommand(
        shooter.runStateCmd(
            () -> Rotation2d.fromDegrees(5.0), () -> flywheelIdleSpeed, () -> flywheelIdleSpeed));
    // reactionBarRelease.setDefaultCommand(
    //     reactionBarRelease.setRotationCmd(Rotation2d.fromDegrees(0.0)));
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
                    leds.setBlinkingCmd(new Color("#ff8000"), new Color("#000000"), 25.0))
                .withTimeout(0.5));

    // ---- Controller bindings here ----
    controller.leftTrigger().whileTrue(intake.runVelocityCmd(80.0, 30.0));
    controller
        .rightTrigger()
        .and(() -> currentTarget == Target.SPEAKER)
        .and(() -> false)
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
                              -SwerveSubsystem.MAX_LINEAR_SPEED / 2,
                              SwerveSubsystem.MAX_LINEAR_SPEED / 2);
                      double polarRadians = Math.atan2(vy, vx);
                      ChassisSpeeds polarSpeeds =
                          new ChassisSpeeds(
                              polarVelocity * Math.cos(polarRadians),
                              polarVelocity * Math.sin(polarRadians),
                              omega);
                      Logger.recordOutput("AutoAim/Polar Speeds", polarSpeeds);
                      return polarSpeeds;
                    }),
                Commands.waitSeconds(0.5)
                    .andThen(feeder.runVoltageCmd(FeederSubsystem.INDEXING_VOLTAGE))));
    controller
        .rightTrigger()
        .and(() -> currentTarget == Target.SPEAKER)
        .and(() -> true)
        .whileTrue(
            Commands.parallel(
                shooter.runStateCmd(Rotation2d.fromDegrees(60.0), 80.0, 60.0),
                Commands.waitSeconds(1.0)
                    .andThen(feeder.runVoltageCmd(FeederSubsystem.INDEXING_VOLTAGE))));
    controller
        .rightTrigger()
        .and(() -> currentTarget == Target.AMP)
        .whileTrue(elevator.setExtensionCmd(() -> ElevatorSubsystem.AMP_EXTENSION_METERS))
        .onFalse(
            Commands.parallel(
                    carriage.runVoltageCmd(-3.0),
                    elevator.setExtensionCmd(() -> ElevatorSubsystem.AMP_EXTENSION_METERS))
                .withTimeout(0.5));
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
    SmartDashboard.putData("Dynamic Demo", dynamicAutoDemo());
    SmartDashboard.putData(
        "gloob",
        Commands.sequence(
            Commands.runOnce(
                () -> swerve.setPose(new Pose2d(0.71, 6.72, Rotation2d.fromRadians(1.04))), swerve),
            startToNote(),
            blacklistNote(),
            noteToShoot(),
            noteToNote()));
    SmartDashboard.putData(
        "gleeb",
        Commands.sequence(
            Commands.runOnce(
                () -> swerve.setPose(new Pose2d(0.71, 6.72, Rotation2d.fromRadians(1.04))), swerve),
            startToNote(),
            blacklistNote(),
            noteToNote()));

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

    SmartDashboard.putData("Dynamic Auto", dynamicAuto());
  }

  public Command dynamicAutoDemo() {
    return Commands.sequence(
            Commands.runOnce(
                () -> swerve.setPose(new Pose2d(0.71, 6.72, Rotation2d.fromRadians(1.04))), swerve),
            startToNote(),
            blacklistNote(),
            noteToShoot(),
            shootToNote(),
            blacklistNote(),
            noteToShoot(),
            shootToNote(),
            blacklistNote(),
            noteToNote())
        .asProxy();
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
            .getPoseAllianceSpicific());
    Logger.recordOutput("DynamicAuto/Curent Trajectory Followed", curTrajectory.getPoses());
    Logger.recordOutput("DynamicAuto/Forward Trajectory Followed", forwardLookingTrajectory);
    // Logger.recordOutput("Canivore Util", CANBus.getStatus("canivore").BusUtilization);
  }

  private LoggedDashboardNumber rotation = new LoggedDashboardNumber("Rotation (Rotations)");
  private LoggedDashboardNumber leftRPS = new LoggedDashboardNumber("Left RPS (Rotations Per Sec)");
  private LoggedDashboardNumber rightRPS =
      new LoggedDashboardNumber("Right RPS (Rotations Per Sec)");

  public Command shootWithDashboard() {
    return shooter.runStateCmd(
        () -> Rotation2d.fromRotations(rotation.get()), () -> leftRPS.get(), () -> rightRPS.get());
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
              AutoAimStates.curShotData =
                  AutoAim.shotMap.get(
                      swerve
                          .getLinearFuturePose(
                              AutoAim.LOOKAHEAD_TIME_SECONDS, AutoAimStates.curShotSpeeds)
                          .minus(FieldConstants.getSpeaker())
                          .getTranslation()
                          .getNorm());
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
        Commands.deadline(Commands.waitSeconds(AutoAim.LOOKAHEAD_TIME_SECONDS), runRobot),
        // keeps moving to prevent the robot from stopping and changing the velocity of the note
        swerve
            .runVelocityFieldRelative(
                () ->
                    new ChassisSpeeds(
                        AutoAimStates.curShotSpeeds.vxMetersPerSecond,
                        AutoAimStates.curShotSpeeds.vyMetersPerSecond,
                        0))
            .withTimeout(0.25));
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

  private LoggedDashboardNumber priority = new LoggedDashboardNumber("Note Priority");
  private LoggedDashboardBoolean blacklist = new LoggedDashboardBoolean("Note Blacklisted");

  public Command updateNote() {
    return Commands.runOnce(
        () ->
            DynamicAuto.updateNote(
                noteDropdown.get(), () -> blacklist.get(), () -> (int) priority.get()));
  }

  public Command startToNote() {
    return swerve
        .runChoreoTraj(
            () -> {
              curTrajectory = DynamicAuto.makeStartToNote(swerve::getPose);
              forwardLookingTrajectory =
                  DynamicAuto.addTwoTrajectories(
                      curTrajectory, DynamicAuto.makeNoteToShooting(curTrajectory::getFinalPose));
              return curTrajectory;
            })
        .onlyIf(
            () -> {
              if (DynamicAuto.whitelistCount > 0) {
                return true;
              } else {
                System.out.println("No more avalible notes!!!!! >:(");
                return false;
              }
            });
  }

  public Command noteToNote() {
    return swerve
        .runChoreoTraj(
            () -> {
              System.out.println(DynamicAuto.whitelistCount);
              curTrajectory = DynamicAuto.makeNoteToNote(swerve::getPose);
              forwardLookingTrajectory =
                  DynamicAuto.addTwoTrajectories(
                      curTrajectory, DynamicAuto.makeNoteToShooting(curTrajectory::getFinalPose));
              return DynamicAuto.makeNoteToNote(swerve::getPose);
            })
        .onlyIf(
            () -> {
              if (DynamicAuto.whitelistCount > 0) {
                return true;
              } else {
                System.out.println("No more avalible notes!!!!! >:(");
                return false;
              }
            });
  }

  public Command shootToNote() {
    return swerve
        .runChoreoTraj(
            () -> {
              curTrajectory = DynamicAuto.makeShootingToNote(swerve::getPose);
              forwardLookingTrajectory =
                  DynamicAuto.addTwoTrajectories(
                      curTrajectory, DynamicAuto.makeNoteToShooting(curTrajectory::getFinalPose));
              return DynamicAuto.makeShootingToNote(swerve::getPose);
            })
        .onlyIf(
            () -> {
              if (DynamicAuto.whitelistCount > 0) {
                return true;
              } else {
                System.out.println("No more avalible notes!!!!! >:(");
                return false;
              }
            });
  }

  public Command noteToShoot() {
    return swerve
        .runChoreoTraj(
            () -> {
              curTrajectory = DynamicAuto.makeNoteToShooting(swerve::getPose);
              return DynamicAuto.makeNoteToShooting(swerve::getPose);
            })
        .onlyIf(
            () -> {
              if (DynamicAuto.whitelistCount > 0) {
                return true;
              } else {
                System.out.println("No more avalible notes!!!!! >:(");
                return false;
              }
            });
  }

  public Command blacklistNote() {
    return Commands.runOnce(() -> DynamicAuto.getAbsoluteClosestNote(swerve::getPose).blacklist());
  }

  public CommandSelector selectAuto() {
    if (dynamicAutoCounter == 0) {
      //  swerve.setPose(new Pose2d(0.71, 6.72, Rotation2d.fromRadians(1.04)));
      System.out.println("start to note");
      return CommandSelector.START_TO_NOTE;

    } else if (atShootingLocation) {
      atShootingLocation = false;
      System.out.println("shoot to note");
      return CommandSelector.SHOOT_TO_NOTE;
    } else if ((carriage.getBeambreak() || feeder.getFirstBeambreak())
        || (DynamicAuto.getAbsoluteClosestNote(swerve::getPose).getExistence()
            && swerve
                    .getPose()
                    .minus(
                        DynamicAuto.getAbsoluteClosestNote(swerve::getPose)
                            .getPoseAllianceSpicific())
                    .getTranslation()
                    .getNorm()
                < 1.5)) {
      System.out.println("note to shoot");
      atShootingLocation = true;
      DynamicAuto.getAbsoluteClosestNote(swerve::getPose).blacklist();
      return CommandSelector.NOTE_TO_SHOOT;
    } else if ((!(carriage.getBeambreak() || feeder.getFirstBeambreak()) && Robot.isReal())
        || (!DynamicAuto.getAbsoluteClosestNote(swerve::getPose).getExistence()
            && swerve
                    .getPose()
                    .minus(
                        DynamicAuto.getAbsoluteClosestNote(swerve::getPose)
                            .getPoseAllianceSpicific())
                    .getTranslation()
                    .getNorm()
                < 1.5)) {

      System.out.println("note to note 1");
      DynamicAuto.getAbsoluteClosestNote(swerve::getPose).blacklist();
      return CommandSelector.NOTE_TO_NOTE;
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
      return CommandSelector.DYNAMIC_TO_NOTE;
    } else {
      atShootingLocation = true;
      System.out.println("dynamoci to shoot");
      return CommandSelector.DYNAMIC_TO_SHOOT;
    }
  }

  double distance = 0;

  public Command dynamicAuto() {

    return Commands.repeatingSequence(
            Commands.select(
                Map.ofEntries(
                    Map.entry(
                        CommandSelector.NOTE_TO_NOTE,
                        Commands.race(noteToNote(), intake.runVelocityCmd(80.0, 30.0))),
                    Map.entry(
                        CommandSelector.NOTE_TO_SHOOT,
                        Commands.race(
                            noteToShoot(),
                            shooter.runStateCmd(
                                () -> AutoAim.shotMap.get(distance).getRotation(),
                                () -> AutoAim.shotMap.get(distance).getLeftRPS(),
                                () -> AutoAim.shotMap.get(distance).getRightRPS()))),
                    Map.entry(
                        CommandSelector.START_TO_NOTE,
                        Commands.race(startToNote(), intake.runVelocityCmd(80.0, 30.0))),
                    Map.entry(
                        CommandSelector.SHOOT_TO_NOTE,
                        Commands.race(shootToNote(), intake.runVelocityCmd(80.0, 30.0))),
                    Map.entry(
                        CommandSelector.DYNAMIC_TO_NOTE,
                        Commands.race(
                            DynamicAuto.DynamicToNote(swerve::getPose),
                            intake.runVelocityCmd(80.0, 30.0))),
                    Map.entry(
                        CommandSelector.DYNAMIC_TO_SHOOT,
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
                      swerve
                          .getPose()
                          .minus(FieldConstants.getSpeaker())
                          .getTranslation()
                          .getNorm();
                }))
        .asProxy();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    dynamicAutoCounter = 0;
    for (Note note : DynamicAuto.notes) {
      note.whitelist();
    }
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
