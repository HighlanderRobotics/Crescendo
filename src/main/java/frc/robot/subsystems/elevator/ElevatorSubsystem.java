// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

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

/** Slanted cascading elevator */
public class ElevatorSubsystem extends SubsystemBase {
  // Constants
  // TODO find real values
  public static final double GEAR_RATIO = 12.5 / 1.0;
  public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.751 / 2.0);
  public static Rotation2d ELEVATOR_ANGLE = Rotation2d.fromDegrees(65.0);

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO io;

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

  public Pose3d getCarriagePose() {
    return new Pose3d(
        Units.inchesToMeters(4.5) + carriage.getLength() * Math.cos(ELEVATOR_ANGLE.getRadians()),
        0.0,
        Units.inchesToMeters(7.0) + carriage.getLength() * Math.sin(ELEVATOR_ANGLE.getRadians()),
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

  public Command setExtensionCmd(DoubleSupplier meters) {
    return this.run(
        () -> {
          io.setTarget(meters.getAsDouble());
          Logger.recordOutput("Elevator/Setpoint", meters.getAsDouble());
        });
  }
}
