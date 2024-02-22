package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.google.common.base.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubystem extends SubsystemBase {
  public static final double PIVOT_RATIO = (27.0 / 1.0) * (48.0 / 22.0);
  public static final double FLYWHEEL_RATIO = 18.0 / 24.0;

  public static final Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromDegrees(8.5);
  public static final Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(106.0);

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;

  private final SysIdRoutine flywheelRoutine;
  private final SysIdRoutine pivotRoutine;

  private final Mechanism2d mech2d =
      new Mechanism2d(Units.feetToMeters(0.0), Units.feetToMeters(4.0));
  private final MechanismRoot2d root =
      mech2d.getRoot("Shooter Root", Units.inchesToMeters(1.7), Units.inchesToMeters(10.8));
  private final MechanismLigament2d shooterLig =
      root.append(new MechanismLigament2d("Shooter", Units.inchesToMeters(13.0), 0.0));

  public ShooterSubystem(ShooterIO shooterIO) {
    this.io = shooterIO;
    inputs = new ShooterIOInputsAutoLogged();

    flywheelRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Default ramp rate is acceptable
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                null, // Default timeout is acceptable
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> io.setFlywheelVoltage(volts.in(Volts), volts.in(Volts)),
                null,
                this));
    pivotRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Default ramp rate is acceptable
                Volts.of(6),
                null, // Default timeout is acceptable
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> io.setPivotVoltage(volts.in(Volts)), null, this));
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
          Logger.recordOutput(
              "Shooter/Left At Target",
              MathUtil.isNear(
                  left.getAsDouble(), inputs.flywheelLeftVelocityRotationsPerSecond, 1.0));
          Logger.recordOutput(
              "Shooter/Right At Target",
              MathUtil.isNear(
                  right.getAsDouble(), inputs.flywheelRightVelocityRotationsPerSecond, 1.0));
          Logger.recordOutput("Shooter/Rotation Setpoint", rotation.get().getRadians());
          Logger.recordOutput(
              "Shooter/Pivot Error Degrees",
              inputs.pivotRotation.getDegrees() - rotation.get().getDegrees());
          Logger.recordOutput(
              "Shooter/Pivot At Target",
              MathUtil.isNear(
                  rotation.get().getDegrees(), inputs.pivotRotation.getDegrees(), 0.25));
          io.setFlywheelVelocity(left.getAsDouble(), right.getAsDouble());
          io.setPivotSetpoint(rotation.get());
        });
  }

  public Command runFlywheelsCmd(DoubleSupplier left, DoubleSupplier right) {
    return this.run(
        () -> {
          Logger.recordOutput("Shooter/Left Velocity Setpoint", left.getAsDouble());
          Logger.recordOutput("Shooter/Right Velocity Setpoint", right.getAsDouble());
          Logger.recordOutput("Shooter/Rotation Setpoint", 0.0);
          Logger.recordOutput(
              "Shooter/Left At Target",
              MathUtil.isNear(
                  left.getAsDouble(), inputs.flywheelLeftVelocityRotationsPerSecond, 1.0));
          Logger.recordOutput(
              "Shooter/Right At Target",
              MathUtil.isNear(
                  right.getAsDouble(), inputs.flywheelRightVelocityRotationsPerSecond, 1.0));
          Logger.recordOutput(
              "Shooter/Pivot At Target",
              MathUtil.isNear(
                  right.getAsDouble(),
                  inputs.flywheelRightVelocityRotationsPerSecond,
                  Units.degreesToRotations(0.5)));
          io.setFlywheelVelocity(left.getAsDouble(), right.getAsDouble());
          io.setPivotVoltage(0.0);
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

  public Command runPivotCurrentZeroing() {
    return this.run(() -> io.setPivotVoltage(-1.0))
        .until(() -> inputs.pivotAmps > 40.0)
        .finallyDo(() -> io.resetPivotPostion(PIVOT_MIN_ANGLE));
  }

  public Command runFlywheelSysidCmd() {
    return Commands.sequence(
        runPivotCurrentZeroing(),
        this.runOnce(() -> SignalLogger.start()),
        flywheelRoutine.quasistatic(Direction.kForward),
        this.runOnce(() -> io.setFlywheelVoltage(0.0, 0.0)),
        Commands.waitSeconds(1.0),
        flywheelRoutine.quasistatic(Direction.kReverse),
        this.runOnce(() -> io.setFlywheelVoltage(0.0, 0.0)),
        Commands.waitSeconds(1.0),
        flywheelRoutine.dynamic(Direction.kForward),
        this.runOnce(() -> io.setFlywheelVoltage(0.0, 0.0)),
        Commands.waitSeconds(1.0),
        flywheelRoutine.dynamic(Direction.kReverse),
        this.runOnce(() -> SignalLogger.stop()));
  }

  public Command runPivotSysidCmd() {
    return Commands.sequence(
        this.runOnce(() -> SignalLogger.start()),
        // Stop when we get close to vertical so it falls back
        pivotRoutine
            .quasistatic(Direction.kForward)
            .until(() -> inputs.pivotRotation.getDegrees() > 80.0),
        this.runOnce(() -> io.setFlywheelVoltage(0.0, 0.0)),
        Commands.waitSeconds(0.25),
        // Stop when near horizontal so we avoid hard stop
        pivotRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> inputs.pivotRotation.getDegrees() < 10.0),
        this.runOnce(() -> io.setFlywheelVoltage(0.0, 0.0)),
        Commands.waitSeconds(0.25),
        // Stop when we get close to vertical so it falls back
        pivotRoutine
            .dynamic(Direction.kForward)
            .until(() -> inputs.pivotRotation.getDegrees() > 80.0),
        this.runOnce(() -> io.setFlywheelVoltage(0.0, 0.0)),
        Commands.waitSeconds(0.25),
        // Stop when near horizontal so we avoid hard stop
        pivotRoutine
            .dynamic(Direction.kReverse)
            .until(() -> inputs.pivotRotation.getDegrees() < 10.0),
        this.runOnce(() -> SignalLogger.stop()));
  }
}
