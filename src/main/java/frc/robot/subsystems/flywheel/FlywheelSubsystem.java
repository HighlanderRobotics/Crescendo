package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {
  public static final double FLYWHEEL_RATIO = 18.0 / 24.0;

  private final FlywheelIO io;
  // We have multiple flywheels, identify them!
  private final String name;
  private final FlywheelIOInputsAutoLogged inputs;

  private double goalRPS = 0.0;

  public FlywheelSubsystem(FlywheelIO io, String name) {
    this.io = io;
    this.name = name;
    inputs = new FlywheelIOInputsAutoLogged();
  }

  public Command setVelocity(final double rps) {
    return this.run(
        () -> {
          io.setVelocity(rps);
          goalRPS = rps;
        });
  }

  public Command setVoltageCmd(final double volts) {
    return this.run(
        () -> {
          io.setVoltage(volts);
          // Estimate goal rps from kv
          goalRPS = (volts / 12.0) * (5800.0 / 60.0);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel " + name, inputs);
  }

  public boolean isAtGoal() {
    return MathUtil.isNear(goalRPS, inputs.velocityRotationsPerSecond, 1.0);
  }
}
