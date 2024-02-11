package frc.robot.subsystems.shooter;

import com.google.common.base.Supplier;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubystem extends SubsystemBase {
  public static final double PIVOT_RATIO = (27.0 / 1.0) * (48.0 / 22.0);
  public static final double FLYWHEEL_RATIO = 18.0 / 24.0;

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;

  private final Mechanism2d mech2d =
      new Mechanism2d(Units.feetToMeters(0.0), Units.feetToMeters(4.0));
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
    Logger.recordOutput("Shooter/Root Pose", getMechanismPose());
  }

  public Pose3d getMechanismPose() {
    return new Pose3d(
        0.0437896, 0.0, 0.3274568, new Rotation3d(0.0, inputs.pivotRotation.getRadians(), 0.0));
  }

  public Command runStateCmd(
      Supplier<Rotation2d> rotation, DoubleSupplier left, DoubleSupplier right) {
    return this.run(
        () -> {
          Logger.recordOutput("Shooter/Left Velocity Setpoint", left.getAsDouble());
          Logger.recordOutput("Shooter/Right Velocity Setpoint", right.getAsDouble());
          Logger.recordOutput("Shooter/Rotation Setpoint", rotation.get());
          io.setFlywheelVelocity(left.getAsDouble(), right.getAsDouble());
          io.setPivotSetpoint(rotation.get());
        });
  }

  public Command runStateCmd(Rotation2d rotation, double left, double right) {
    return runStateCmd(() -> rotation, () -> left, () -> right);
  }

  public Command runFlywheelVoltageCmd(Rotation2d rotation, double voltage) {
    return this.run(
        () -> {
          io.setFlywheelVoltage(voltage, voltage);
          io.setPivotSetpoint(new Rotation2d());
        });
  }
}
