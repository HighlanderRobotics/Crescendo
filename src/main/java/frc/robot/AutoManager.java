package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class AutoManager {
  LoggedDashboardChooser<Supplier<Command>> chooser =
      new LoggedDashboardChooser<Supplier<Command>>("Auto Chooser");

  SwerveSubsystem swerveSubsystem;
  IntakeSubsystem intakeSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  ShooterSubystem shooterSubsystem;
  FeederSubsystem feederSubsystem;

  public AutoManager(
      SwerveSubsystem swerveSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ShooterSubystem shooterSubsystem,
      FeederSubsystem feederSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
  }
}
