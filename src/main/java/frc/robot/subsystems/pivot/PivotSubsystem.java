// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {
  public static final double RATIO = (27.0 / 1.0) * (48.0 / 22.0);

  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(14.951);
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(106.0);

  private final PivotIO io;
  public final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private Rotation2d goal = MIN_ANGLE;

  private final Mechanism2d mech2d =
      new Mechanism2d(Units.feetToMeters(0.0), Units.feetToMeters(4.0));
  private final MechanismRoot2d root =
      mech2d.getRoot("Shooter Root", Units.inchesToMeters(1.7), Units.inchesToMeters(10.8));
  private final MechanismLigament2d shooterLig =
      root.append(new MechanismLigament2d("Shooter", Units.inchesToMeters(13.0), 0.0));

  public PivotSubsystem(PivotIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Pivot", inputs);

    shooterLig.setAngle(inputs.pivotRotation.unaryMinus().minus(Rotation2d.fromDegrees(180.0)));
    Logger.recordOutput("Shooter/Mechanism2d", mech2d);
    Logger.recordOutput("Shooter/Root Pose", getMechanismPose());
  }

  public Pose3d getMechanismPose() {
    return new Pose3d(
        0.0437896, 0.0, 0.3274568, new Rotation3d(0.0, inputs.pivotRotation.getRadians(), 0.0));
  }

  public Rotation2d getAngle() {
    return inputs.pivotRotation;
  }

  public Command setPivotSetpoint(Rotation2d goal) {
    return this.run(
        () -> {
          this.goal = goal;
          io.setPivotSetpoint(goal);
        });
  }

  public Command setMinCmd() {
    return this.run(
        () -> {
          this.goal = MIN_ANGLE;
          io.setPivotSetpoint(MIN_ANGLE);
        });
  }

  @AutoLogOutput(key = "Shooter/Pivot/At Goal")
  public boolean isAtGoal() {
    return MathUtil.isNear(goal.getDegrees(), inputs.pivotRotation.getDegrees(), 0.5)
        && MathUtil.isNear(0.0, inputs.pivotVelocityRotationsPerSecond, 0.05);
  }

  public Command resetPosition(Rotation2d position) {
    return this.runOnce(() -> io.resetPivotPosition(position));
  }
}
