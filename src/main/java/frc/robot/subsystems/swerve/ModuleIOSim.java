// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.DriveTrainConstants;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import java.util.Arrays;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);

  private final ModuleConstants constants;

  // must be public for sim
  public final SwerveModulePhysicsSimulationResults driveSimResults =
      new SwerveModulePhysicsSimulationResults();
  ;
  private final DCMotorSim turnSim =
      // Third param is the moment of inertia of the swerve steer
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), Module.TURN_GEAR_RATIO, 0.004);

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final PIDController turnController = new PIDController(100.0, 0.0, 0.0);
  private final PIDController driveController = new PIDController(0.3, 0.0, 0.0);
  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 2.0);

  public ModuleIOSim(final ModuleConstants constants) {
    this.constants = constants;
  }

  @Override
  public void updateInputs(final ModuleIOInputs inputs) {
    inputs.prefix = constants.prefix();

    inputs.drivePositionMeters =
        driveSimResults.driveWheelFinalRevolutions * 2 * Math.PI * Module.WHEEL_RADIUS_METERS;
    inputs.driveVelocityMetersPerSec =
        driveSimResults.driveWheelFinalVelocityRadPerSec * Module.WHEEL_RADIUS_METERS;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps =
        new double[] {Math.abs(getSimulationTorque() / DRIVE_MOTOR.KtNMPerAmp)};
    inputs.driveSupplyCurrentAmps =
        Math.abs(
            (driveAppliedVolts / RoboRioSim.getVInVoltage())
                * (getSimulationTorque() / DRIVE_MOTOR.KtNMPerAmp));

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
  }

  @Override
  public void setDriveVoltage(final double volts, final boolean focEnabled) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setTurnVoltage(final double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveSetpoint(final double metersPerSecond, final double metersPerSecondSquared) {
    setDriveVoltage(
        driveController.calculate(
                driveSimResults.driveWheelFinalVelocityRadPerSec * Module.WHEEL_RADIUS_METERS,
                metersPerSecond)
            + driveFeedforward.calculate(metersPerSecond, metersPerSecondSquared));
  }

  @Override
  public void setTurnSetpoint(final Rotation2d rotation) {
    setTurnVoltage(
        turnController.calculate(turnSim.getAngularPositionRotations(), rotation.getRotations()));
  }

  public void updateSim(double dt) {
    turnSim.update(dt);
  }

  public double getSimulationTorque() {
    return DRIVE_MOTOR.getTorque(
        DRIVE_MOTOR.getCurrent(
            driveSimResults.driveWheelFinalVelocityRadPerSec * Module.DRIVE_GEAR_RATIO,
            driveAppliedVolts));
  }

  public Rotation2d getSimulationSteerFacing() {
    return Rotation2d.fromRadians(turnSim.getAngularPositionRad());
  }

  public SwerveModuleState getSimulationSwerveState() {
    return new SwerveModuleState(
        driveSimResults.driveWheelFinalVelocityRadPerSec * Module.WHEEL_RADIUS_METERS,
        getSimulationSteerFacing());
  }

  public SwerveModuleState getDesiredSwerveState() {
    return new SwerveModuleState(
        (driveAppliedVolts / 12.0) * SwerveSubsystem.MAX_LINEAR_SPEED, getSimulationSteerFacing());
  }

  /** this replaces DC Motor Sim for drive wheels from maple template */
  public static class SwerveModulePhysicsSimulationResults {
    public double driveWheelFinalRevolutions = 0, driveWheelFinalVelocityRadPerSec = 0;

    public final double[] odometryDriveWheelRevolutions =
        new double[DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD];
    public final Rotation2d[] odometrySteerPositions =
        new Rotation2d[DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD];

    public SwerveModulePhysicsSimulationResults() {
      Arrays.fill(odometrySteerPositions, new Rotation2d());
      Arrays.fill(odometryDriveWheelRevolutions, 0);
    }
  }
}
