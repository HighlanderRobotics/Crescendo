package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubystem extends SubsystemBase {
  public static final double PIVOT_RATIO = 35.0 / 1.0;
  public static final double FLYWHEEL_RATIO = 18.0 / 24.0;

  private final ShooterIO io;
  private ShooterIOInputsAutoLogged inputs;

  Mechanism2d mech2d = new Mechanism2d(Units.feetToMeters(0.0), Units.feetToMeters(4.0));
  MechanismRoot2d root = mech2d.getRoot("Shooter Root", Units.inchesToMeters(1.7), Units.inchesToMeters(10.8));
  MechanismLigament2d shooterLig =
      root.append(new MechanismLigament2d("Shooter", Units.inchesToMeters(13.0), 0.0));

  public ShooterSubystem(ShooterIO pivotIO) {
    this.io = pivotIO;
    inputs = new ShooterIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();
    Logger.processInputs("Shooter", inputs);

    shooterLig.setAngle(inputs.pivotRotation.unaryMinus().minus(Rotation2d.fromDegrees(180.0)));
    Logger.recordOutput("Shooter/Mechanism2d", mech2d);
  }

  public Command runStateCmd(Rotation2d rotation, double left, double right) {
    return this.run(
        () -> {
          Logger.recordOutput("Shooter/Left Velocity Setpoint", left);
          Logger.recordOutput("Shooter/Right Velocity Setpoint", right);
          Logger.recordOutput("Shooter/Rotation Setpoint", rotation);
          io.setFlywheelVelocity(left, right);
          io.setPivotSetpoint(rotation);
        });
  }

  public Command runFlywheelVoltageCmd(Rotation2d rotation, double voltage) {
    return this.run(
        () -> {
          io.setFlywheelVoltage(voltage, voltage);
          io.setPivotSetpoint(new Rotation2d());
        });
  }
}
