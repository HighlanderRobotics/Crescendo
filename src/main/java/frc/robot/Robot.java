// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.routing.RoutingIOReal;
import frc.robot.subsystems.routing.RoutingSubsystem;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
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
          mode == RobotMode.REAL ? new GyroIOPigeon2() : new GyroIO() {},
          mode == RobotMode.REAL
              ? SwerveSubsystem.createTalonFXModules()
              : SwerveSubsystem.createSimModules());
  private final ShooterSubsystem shooter = new ShooterSubsystem(new ShooterIOReal());
  private final PivotSubsystem pivot = new PivotSubsystem(new PivotIOReal());
  private final RoutingSubsystem routing = new RoutingSubsystem(new RoutingIOReal());

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

    shooter.setDefaultCommand(shooter.run(0.0));
    pivot.setDefaultCommand(pivot.run(-100.0));
    routing.setDefaultCommand(routing.run(0.0));

    // Controller bindings here
    controller.start().onTrue(Commands.runOnce(() -> swerve.setYaw(Rotation2d.fromDegrees(0))));

    controller.leftTrigger().whileTrue(intake());
    controller.rightTrigger().whileTrue(shootFender());
    controller
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                pivot.run(10.0),
                shooter.run(-2.0),
                Commands.waitSeconds(0.5).andThen(routing.run(-6.0 * 360))));
    controller
        .a()
        .onTrue(swerve.runOnce(() -> swerve.setPose(new Pose2d(2.0, 2.0, new Rotation2d()))));
    // Auto Bindings here
    NamedCommands.registerCommand("fender", shootFender());
    NamedCommands.registerCommand("intake", intake());
    NamedCommands.registerCommand("stop", swerve.stopWithXCmd().asProxy());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = new PathPlannerAuto("local 4");

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

  private Command intake() {
    return Commands.parallel(shooter.run(5.0), pivot.run(0.0));
  }

  private Command shootFender() {
    return Commands.parallel(
            swerve.stopCmd(),
            pivot.run(-160.0),
            Commands.waitSeconds(0.5).andThen(shooter.run(-40.0)),
            Commands.waitSeconds(1.0).andThen(routing.run(-40.0).asProxy()))
        .withTimeout(1.5)
        .andThen(Commands.print("done shooting"));
  }
}
