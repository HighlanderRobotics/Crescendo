package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubystem extends SubsystemBase {
  public static final double PIVOT_RATIO = 35.0 / 1.0;
  public static final double FLYWHEEL_RATIO = 18.0 / 24.0;

  ShooterIO io;
  ShooterIOInputsAutoLogged inputs;

  public ShooterSubystem(ShooterIO pivotIO) {
    this.io = pivotIO;
    inputs = new ShooterIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();
    Logger.processInputs("Shooter", inputs);
  }

  public Command runStateCmd(Rotation2d rotation, double rps) {
    return this.run(
        () -> {
          io.setFlywheelVelocity(rps);
          io.setPivotSetpoint(rotation);
        });
  }

  public Command runFlywheelvoltageCmd(Rotation2d rotation, double voltage) {
    return this.run(
        () -> {
          io.setFlywheelVoltage(voltage);
          io.setPivotSetpoint(new Rotation2d());
        });
  }
}
