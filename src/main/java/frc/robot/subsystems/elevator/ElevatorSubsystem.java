// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
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

/** Slanted cascading elevator */
public class ElevatorSubsystem extends SubsystemBase {
  // Constants
  // TODO find real values
  public static final double GEAR_RATIO = 12.5 / 1.0;
  public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.751 / 2.0);
  public static final Rotation2d ELEVATOR_ANGLE = Rotation2d.fromDegrees(65.0);

  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(31.3);

  public static final double CLIMB_EXTENSION_METERS = 0.6;
  public static final double TRAP_EXTENSION_METERS = 0.9;
  public static final double AMP_EXTENSION_METERS = 0.6;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO io;

  private final SysIdRoutine elevatorRoutine;

  // For dashboard
  private final Mechanism2d mech2d = new Mechanism2d(3.0, Units.feetToMeters(4.0));
  private final MechanismRoot2d
      root = // CAD distance from origin to center of carriage at full retraction
      mech2d.getRoot(
              "Elevator", (3.0 / 2.0) + Units.inchesToMeters(9.053), Units.inchesToMeters(12.689));
  private final MechanismLigament2d carriage =
      new MechanismLigament2d("Carriage", 0, ELEVATOR_ANGLE.getDegrees());

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;

    elevatorRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Default ramp rate is acceptable
                Volts.of(4),
                null, // Default timeout is acceptable
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> io.setVoltage(volts.in(Volts)), null, this));

    root.append(carriage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    carriage.setLength(inputs.elevatorPositionMeters);
    Logger.recordOutput("Elevator/Mechanism2d", mech2d);

    Logger.recordOutput("Elevator/Carriage Pose", getCarriagePose());
  }

  public Command setExtensionCmd(DoubleSupplier meters) {
    return this.run(
        () -> {
          io.setTarget(meters.getAsDouble());
          Logger.recordOutput("Elevator/Setpoint", meters.getAsDouble());
        });
  }

  public Command runCurrentZeroing() {
    return this.run(() -> io.setVoltage(-1.0))
        .until(() -> inputs.elevatorCurrentAmps[0] > 40.0)
        .finallyDo(() -> io.resetEncoder(0.0));
  }

  public Command runSysidCmd() {
    return Commands.sequence(
        runCurrentZeroing(),
        this.runOnce(() -> SignalLogger.start()),
        // Stop when we get close to max to avoid hitting hard stop
        elevatorRoutine
            .quasistatic(Direction.kForward)
            .until(() -> inputs.elevatorPositionMeters > MAX_EXTENSION_METERS - 0.2),
        this.runOnce(() -> io.setVoltage(0.0)),
        Commands.waitSeconds(1.0),
        // Stop when we get close to max to avoid hitting hard stop
        elevatorRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> inputs.elevatorPositionMeters < 0.2),
        this.runOnce(() -> io.setVoltage(0.0)),
        Commands.waitSeconds(1.0),
        // Stop when we get close to max to avoid hitting hard stop
        elevatorRoutine
            .dynamic(Direction.kForward)
            .until(() -> inputs.elevatorPositionMeters > MAX_EXTENSION_METERS - 0.2),
        this.runOnce(() -> io.setVoltage(0.0)),
        Commands.waitSeconds(1.0),
        // Stop when we get close to max to avoid hitting hard stop
        elevatorRoutine
            .dynamic(Direction.kReverse)
            .until(() -> inputs.elevatorPositionMeters < 0.2),
        this.runOnce(() -> SignalLogger.stop()));
  }

  public Pose3d getCarriagePose() {
    return new Pose3d(
        Units.inchesToMeters(4.5) + carriage.getLength() * ELEVATOR_ANGLE.getCos(),
        0.0,
        Units.inchesToMeters(7.0) + carriage.getLength() * ELEVATOR_ANGLE.getSin(),
        new Rotation3d());
  }

  public Pose3d getFirstStagePose() {
    return new Pose3d(
        Units.inchesToMeters(2.25)
            + (carriage.getLength() / 2.0) * Math.cos(ELEVATOR_ANGLE.getRadians()),
        0.0,
        Units.inchesToMeters(4.25)
            + (carriage.getLength() / 2.0) * Math.sin(ELEVATOR_ANGLE.getRadians()),
        new Rotation3d());
  }

  public double getExtensionMeters() {
    return inputs.elevatorPositionMeters;
  }
}
