package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class KickerSubsystem extends SubsystemBase {
  KickerIO io;
  KickerIOInputsAutoLogged inputs;

  public KickerSubsystem(KickerIO io) {
    this.io = io;
    inputs = new KickerIOInputsAutoLogged();

    SmartDashboard.putData(
        "reset Kicker value",
        new InstantCommand(
            () -> {
              io.reset(0);
            }));
  }

  public Command reset() {
    return this.runOnce(
        () -> {
          io.reset(0);
        });
  }

  public RunCommand run(double degrees) {
    return new RunCommand(
        () -> {
          io.setPosition(degrees);
        },
        this);
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();
    Logger.processInputs("Kicker", inputs);
  }
}
