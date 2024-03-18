// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.utils.logging.TalonFXLogger;

/** Elevator IO using TalonFXs. */
public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX motor = new TalonFX(16, "canivore");
  private final TalonFX follower = new TalonFX(17, "canivore");

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage positionVoltage =
      new MotionMagicVoltage(0.0).withEnableFOC(true);

  private final TalonFXLogger leaderLogger = new TalonFXLogger(motor);

  private final TalonFXLogger followerLogger = new TalonFXLogger(follower);

  public ElevatorIOReal() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kG = 0.11591;
    config.Slot0.kS = 0.16898;
    config.Slot0.kV = 11.3;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 69.785;
    config.Slot0.kD = 17.53;

    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicAcceleration = 4.0;
    // Estimated from slightly less than motor free speed
    config.MotionMagic.MotionMagicCruiseVelocity =
        50.0 / (ElevatorSubsystem.GEAR_RATIO * 2 * Math.PI * ElevatorSubsystem.DRUM_RADIUS_METERS);

    // Carriage position meters in direction of elevator
    config.Feedback.SensorToMechanismRatio =
        ElevatorSubsystem.GEAR_RATIO / (2 * Math.PI * ElevatorSubsystem.DRUM_RADIUS_METERS);

    motor.getConfigurator().apply(config);
    motor.setPosition(0.0); // Assume we boot 0ed
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(motor.getDeviceID(), true));
  }

  @Override
  public void updateInputs(final ElevatorIOInputsAutoLogged inputs) {
    inputs.leader = leaderLogger.update();
    inputs.follower = followerLogger.update();
  }

  @Override
  public void setTarget(final double meters) {
    motor.setControl(positionVoltage.withPosition(meters));
  }

  @Override
  public void setVoltage(final double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void resetEncoder(final double position) {
    motor.setPosition(position);
  }
}
