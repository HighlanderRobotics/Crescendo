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

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.swerve.Module.ModuleConstants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final ModuleConstants constants;

  private static final DCMotor driveMotor = DCMotor.getKrakenX60Foc(1);
  private final TalonFX driveTalon;
  private final VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC driveControlVelocity =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1);
  private final DCMotorSim driveSim =
      // Third param is the moment of inertia of the swerve wheel
      // Used to approximate the robot inertia, not perfect but fine for the
      // Fidelity of simulation we are targeting
      new DCMotorSim(driveMotor, Module.DRIVE_GEAR_RATIO, 0.025);
  private final DCMotorSim turnSim =
      // Third param is the moment of inertia of the swerve steer
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), Module.TURN_GEAR_RATIO, 0.004);

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double turnAppliedVolts = 0.0;

  private final PIDController turnController = new PIDController(100.0, 0.0, 0.0);

  public ModuleIOSim(final ModuleConstants constants) {
    this.constants = constants;
    driveTalon = new TalonFX(constants.driveID());
    driveTalon.getConfigurator().apply(ModuleIOReal.DRIVE_CONFIG);
  }

  @Override
  public void updateInputs(final ModuleIOInputs inputs) {
    final var driveSimState = driveTalon.getSimState();
    driveSimState.Orientation = ChassisReference.Clockwise_Positive;
    // driveSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
    driveSim.setInput(driveSimState.getMotorVoltage());

    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);
    driveSimState.setRotorVelocity(
        (driveSim.getAngularVelocityRPM() / 60.0) * Module.DRIVE_GEAR_RATIO);

    inputs.prefix = constants.prefix();

    inputs.drivePositionMeters = driveSim.getAngularPositionRad() * Module.WHEEL_RADIUS;
    inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * Module.WHEEL_RADIUS;
    inputs.driveAppliedVolts = driveSimState.getMotorVoltage();
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
  }

  @Override
  public void setDriveVoltage(final double volts, final boolean focEnabled) {
    driveTalon.setControl(driveVoltage.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  public void setTurnVoltage(final double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveSetpoint(final double metersPerSecond, final double forceNewtons) {
    driveTalon.setControl(
        driveControlVelocity.withVelocity(metersPerSecond).withFeedForward(forceNewtons));
  }

  @Override
  public void setTurnSetpoint(final Rotation2d rotation) {
    setTurnVoltage(
        turnController.calculate(turnSim.getAngularPositionRotations(), rotation.getRotations()));
  }
}
