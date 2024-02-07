package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
  private final ShooterIOInputsAutoLogged inputs;

  private final Mechanism2d mech2d = new Mechanism2d(Units.feetToMeters(0.0), Units.feetToMeters(4.0));
  private final MechanismRoot2d root =
      mech2d.getRoot("Shooter Root", Units.inchesToMeters(1.7), Units.inchesToMeters(10.8));
  private final MechanismLigament2d shooterLig =
      root.append(new MechanismLigament2d("Shooter", Units.inchesToMeters(13.0), 0.0));

  public ShooterSubystem(ShooterIO pivotIO) {
    this.io = pivotIO;
    inputs = new ShooterIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    shooterLig.setAngle(inputs.pivotRotation.unaryMinus().minus(Rotation2d.fromDegrees(180.0)));
    Logger.recordOutput("Shooter/Mechanism2d", mech2d);
    Logger.recordOutput(
        "Shooter/Root Pose",
        new Pose3d(
            0.0437896,
            0.0,
            0.3274568,
            new Rotation3d(0.0, inputs.pivotRotation.getRadians(), 0.0)));
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
