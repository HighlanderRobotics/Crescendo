package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  ShooterIO io;
  ShooterIOInputsAutoLogged inputs;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();
  }

  public Command run(double velocity) {
    return new RunCommand(
        () -> {
          io.setVelocity(velocity);
        },
        this);
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();
    Logger.processInputs("Shooter", inputs);
  }
}
