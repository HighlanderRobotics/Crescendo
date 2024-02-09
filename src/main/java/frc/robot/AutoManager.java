package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public final class AutoManager {
  SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();

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

    NamedCommands.registerCommand("intake", intakeSubsystem.getDefaultCommand());
    NamedCommands.registerCommand("fender", shooterSubsystem.getDefaultCommand());

    SmartDashboard.putData(chooser);
  }
}
