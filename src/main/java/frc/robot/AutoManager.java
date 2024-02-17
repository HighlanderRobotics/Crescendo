package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class AutoManager {
  LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<Command>("Auto Chooser");

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

  public void setUpChooser(String defaultAutoName) {
    registerNamedCommands();

    List<String> autoNames = AutoBuilder.getAllAutoNames();

    List<PathPlannerAuto> options = new ArrayList<>();

    for (String autoName : autoNames) {
      try {
        options.add(new PathPlannerAuto(autoName));
      } catch (Exception e) {
        System.out.println("Error loading auto: " + autoName);
        e.printStackTrace();
      }
    }

    if (!defaultAutoName.isEmpty()) {
      chooser.addDefaultOption(defaultAutoName, new PathPlannerAuto(defaultAutoName));
    }

    options.forEach(auto -> chooser.addOption(auto.getName(), auto));
  }

  private void registerNamedCommands() {
    // NamedCommands.registerCommand("intake", intakeSubsystem.getDefaultCommand());
    // NamedCommands.registerCommand("fender", shooterSubsystem.getDefaultCommand());
    NamedCommands.registerCommand("stop", swerveSubsystem.stopWithXCmd().asProxy());
  }
}
