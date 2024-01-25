package frc.robot.subsystems.routing;

import edu.wpi.first.wpilibj2.command.Command;
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
    io.updateInputs(inputs);
    Logger.processInputs("Routing", inputs);
  }
}
