// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.carriage.CarriageIOReal;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDIOReal;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.reaction_bar_release.ReactionBarReleaseIOReal;
import frc.robot.subsystems.reaction_bar_release.ReactionBarReleaseSubsystem;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubystem;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.AutoAimStates;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.autoaim.AutoAim;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
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
  private Command autonomousCommand;

  private final CommandXboxControllerSubsystem controller = new CommandXboxControllerSubsystem(0);
  private final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  private Target currentTarget = Target.SPEAKER;
  private double flywheelIdleSpeed = 1.0;

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
  private final ElevatorSubsystem elevator = new ElevatorSubsystem(new ElevatorIOSim());
  private final ShooterSubystem shooter =
      new ShooterSubystem(mode == RobotMode.REAL ? new ShooterIOReal() : new ShooterIOSim());
  private final CarriageSubsystem carriage = new CarriageSubsystem(new CarriageIOReal());
  private final ReactionBarReleaseSubsystem reactionBarRelease =
      new ReactionBarReleaseSubsystem(new ReactionBarReleaseIOReal());
  private final LEDSubsystem leds = new LEDSubsystem(new LEDIOReal());

  @Override
  public void robotInit() {
    // Metadata about the current code running on the robot
    Logger.recordMetadata("Codebase", "Comp2024");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
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
        // Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
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
    feeder.setDefaultCommand(feeder.runVoltageCmd(0.0));
    carriage.setDefaultCommand(carriage.runVoltageCmd(0.0));
    intake.setDefaultCommand(intake.runVoltageCmd(10.0));
    shooter.setDefaultCommand(
        shooter.runStateCmd(
            () -> Rotation2d.fromDegrees(0.0), () -> flywheelIdleSpeed, () -> flywheelIdleSpeed));
    reactionBarRelease.setDefaultCommand(
        reactionBarRelease.setRotationCmd(Rotation2d.fromDegrees(0.0)));
    leds.setDefaultCommand(
        leds.defaultStateDisplay(
            () -> DriverStation.isEnabled(), () -> currentTarget == Target.SPEAKER));

    controller.setDefaultCommand(controller.rumbleCmd(0.0, 0.0));
    operator.setDefaultCommand(operator.rumbleCmd(0.0, 0.0));

    // Robot state management bindings
    new Trigger(() -> carriage.getBeambreak() || feeder.getFirstBeambreak())
        .debounce(0.25)
        .whileTrue(
            Commands.parallel(
                intake.runVoltageCmd(0.0).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
                controller.rumbleCmd(1.0, 1.0).withTimeout(0.25),
                leds.setBlinkingCmd(new Color("#ff8000"), new Color("#000000"), 25.0)));
    new Trigger(() -> currentTarget == Target.SPEAKER)
        .whileTrue(Commands.parallel(carriage.runVoltageCmd(5.0), feeder.indexCmd()));
    new Trigger(() -> currentTarget == Target.AMP)
        .whileTrue(
            Commands.either(
                Commands.parallel(carriage.indexBackwardsCmd(), feeder.runVoltageCmd(-5.0)),
                Commands.parallel(carriage.indexForwardsCmd(), feeder.runVoltageCmd(0.0)),
                () -> feeder.getFirstBeambreak()));

    // ---- Controller bindings here ----
    controller.leftTrigger().whileTrue(intake.runVoltageCmd(10.0));
    controller
        .rightTrigger()
        .and(() -> currentTarget == Target.SPEAKER)
        .whileTrue(
            Commands.parallel(
                shooter.runStateCmd(Rotation2d.fromDegrees(80.0), 50.0, 40.0),
                Commands.waitSeconds(0.5).andThen(feeder.runVoltageCmd(3.0))));
    controller
        .rightTrigger()
        .and(() -> currentTarget == Target.SPEAKER)
        .whileTrue(
            Commands.parallel(
                teleopAutoAim(
                    () -> {
                      double vx = swerve.getVelocity().vxMetersPerSecond;
                      double vy = swerve.getVelocity().vyMetersPerSecond;
                      double vTheta = swerve.getVelocity().omegaRadiansPerSecond;

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
                              vTheta);
                      Logger.recordOutput("AutoAim/Polar Speeds", polarSpeeds);
                      return polarSpeeds;
                    }),
                Commands.waitSeconds(0.5).andThen(feeder.runVoltageCmd(3.0))));
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
    controller.b().whileTrue(elevator.setExtensionCmd(() -> 0.5));
    controller.x().whileTrue(elevator.setExtensionCmd(() -> Units.inchesToMeters(30.0)));

    controller
        .y()
        .and(() -> elevator.getExtensionMeters() > 0.9 * ElevatorSubsystem.CLIMB_EXTENSION_METERS)
        .onTrue(
            Commands.sequence(
                elevator
                    .setExtensionCmd(() -> 0.0)
                    .until(() -> elevator.getExtensionMeters() < 0.05),
                Commands.waitUntil(() -> controller.y().getAsBoolean()),
                Commands.parallel(
                    carriage
                        .runVoltageCmd(-CarriageSubsystem.INDEXING_VOLTAGE)
                        .withTimeout(0.25)
                        .andThen(
                            Commands.waitUntil(
                                () ->
                                    elevator.getExtensionMeters()
                                        > 0.95 * ElevatorSubsystem.TRAP_EXTENSION_METERS),
                            carriage.runVoltageCmd(-CarriageSubsystem.INDEXING_VOLTAGE)),
                    elevator.setExtensionCmd(() -> ElevatorSubsystem.TRAP_EXTENSION_METERS))));

    // Prep climb
    operator
        .rightTrigger(0.5)
        .and(operator.rightBumper())
        .toggleOnFalse(
            Commands.parallel(
                elevator.setExtensionCmd(() -> ElevatorSubsystem.CLIMB_EXTENSION_METERS)));
    operator.leftTrigger().onTrue(Commands.runOnce(() -> currentTarget = Target.SPEAKER));
    operator.leftBumper().onTrue(Commands.runOnce(() -> currentTarget = Target.AMP));
    operator.a().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = 1.0));
    operator.b().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = 20.0));
    operator.x().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = 20.0));
    operator.y().onTrue(Commands.runOnce(() -> flywheelIdleSpeed = 80.0));

    SmartDashboard.putData("Shooter shoot", shootWithDashboard());

    NamedCommands.registerCommand("stop", swerve.stopWithXCmd().asProxy());
    NamedCommands.registerCommand(
        "auto aim amp 4 local sgmt 1", autonomousAutoAim("amp 4 local sgmt 1"));

    controller
        .leftBumper()
        .whileTrue(
            teleopAutoAim(
                () -> {
                  double vx = swerve.getVelocity().vxMetersPerSecond;
                  double vy = swerve.getVelocity().vyMetersPerSecond;
                  double vTheta = swerve.getVelocity().omegaRadiansPerSecond;

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
                          vTheta);
                  Logger.recordOutput("AutoAim/Polar Sppeeds", polarSpeeds);
                  return polarSpeeds;
                }));

    // Dashboard command buttons
    SmartDashboard.putData("Run Swerve Azimuth Sysid", swerve.runModuleSteerCharacterizationCmd());
    SmartDashboard.putData("Run Swerve Drive Sysid", swerve.runDriveCharacterizationCmd());
    SmartDashboard.putData("Run Elevator Sysid", elevator.runSysidCmd());
    SmartDashboard.putData("Run Pivot Sysid", shooter.runPivotSysidCmd());
    SmartDashboard.putData("Run Flywheel Sysid", shooter.runFlywheelSysidCmd());
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

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = new PathPlannerAuto("New Auto");

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
