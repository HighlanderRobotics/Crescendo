// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  private AutoManager autoManager = new AutoManager(swerve, intake, elevator, shooter, feeder);
  ;

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
            swerve.pointTowardsTranslation(
                () -> -controller.getLeftY() * SwerveSubsystem.MAX_LINEAR_SPEED,
                () -> -controller.getLeftX() * SwerveSubsystem.MAX_LINEAR_SPEED));

    // Test binding for elevator
    controller.b().whileTrue(elevator.setExtensionCmd(() -> 1.0));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autoManager.chooser.get().schedule();
  }

  @Override
  public void teleopInit() {
    autoManager.chooser.get().cancel();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
