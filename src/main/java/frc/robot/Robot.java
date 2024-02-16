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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.carriage.CarriageIOReal;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubystem;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.AutoAimStates;
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

  public static final RobotMode mode = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;
  private Command autonomousCommand;

  private final CommandXboxController controller = new CommandXboxController(0);

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
                    -controller.getLeftY() * SwerveSubsystem.MAX_LINEAR_SPEED,
                    -controller.getLeftX() * SwerveSubsystem.MAX_LINEAR_SPEED,
                    -controller.getRightX() * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    elevator.setDefaultCommand(elevator.setExtensionCmd(() -> 0.0));
    feeder.setDefaultCommand(feeder.runVoltageCmd(0.0));
    intake.setDefaultCommand(intake.runVoltageCmd(10.0));
    shooter.setDefaultCommand(shooter.runStateCmd(Rotation2d.fromDegrees(0.0), 0.0, 0.0));

    // Controller bindings here
    controller
        .rightTrigger()
        .whileTrue(shooter.runStateCmd(Rotation2d.fromDegrees(45.0), 50.0, 40.0));
    controller.start().onTrue(Commands.runOnce(() -> swerve.setYaw(Rotation2d.fromDegrees(0))));

    // Test binding for autoaim
    controller
        .a()
        .whileTrue(
            swerve.teleopPointTowardsTranslationCmd(
                () -> -controller.getLeftY() * SwerveSubsystem.MAX_LINEAR_SPEED,
                () -> -controller.getLeftX() * SwerveSubsystem.MAX_LINEAR_SPEED));
    // Test binding for elevator
    controller.b().whileTrue(elevator.setExtensionCmd(() -> 0.5));
    controller.x().whileTrue(elevator.setExtensionCmd(() -> Units.inchesToMeters(30.0)));

    SmartDashboard.putData("Shooter shoot", shootWithDashboard());

    NamedCommands.registerCommand("stop", swerve.stopWithXCmd().asProxy());
    NamedCommands.registerCommand(
        "auto aim amp 4 local sgmt 1", autonomousAutoAim("amp 4 local sgmt 1"));
    controller.y().onTrue(new InstantCommand(swerve::getLinearFuturePose));

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
  private LoggedDashboardNumber leftRPS =
      new LoggedDashboardNumber("Left RPS (Rotations Per Sec)");
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
            Commands.deadline(Commands.waitSeconds(AutoAim.LOOKAHEAD_TIME_SECONDS), runRobot)
                .finallyDo(
                    () -> {
                      System.out.println("Shoot note");
                    }),
            // keeps moving to prevent the robot from stopping and changing the velocity of the note
            swerve
                .runVelocityFieldRelative(
                    () ->
                        new ChassisSpeeds(
                            AutoAimStates.curShotSpeeds.vxMetersPerSecond,
                            AutoAimStates.curShotSpeeds.vyMetersPerSecond,
                            0))
                .withTimeout(0.25))
        .andThen(() -> System.out.println(Timer.getFPGATimestamp()), swerve);
  }

  public Command autonomousAutoAim(String pathName) {

    return Commands.sequence(
        Commands.deadline(
                Commands.waitSeconds(AutoAim.LOOKAHEAD_TIME_SECONDS),
                Commands.parallel(
                    shooter.runStateCmd(
                        () -> AutoAimStates.curShotData.getRotation(),
                        () -> AutoAimStates.curShotData.getLeftRPS(),
                        () -> AutoAimStates.curShotData.getRightRPS()),
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
}
