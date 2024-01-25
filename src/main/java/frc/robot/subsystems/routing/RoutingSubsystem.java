package frc.robot.subsystems.routing;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class RoutingSubsystem extends SubsystemBase {
  RoutingIO io;
  RoutingIOInputsAutoLogged inputs;

  public RoutingSubsystem(RoutingIO io) {
    this.io = io;
    inputs = new RoutingIOInputsAutoLogged();
  }

  public Command run(double velocity) {
    return this.run(
        () -> {
          io.setVelocity(velocity);
        });
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();
    Logger.processInputs("Routing", inputs);
  }
}
