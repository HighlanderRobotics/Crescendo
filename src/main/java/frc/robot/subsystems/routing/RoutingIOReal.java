package frc.robot.subsystems.routing;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.routing.RoutingIOInputsAutoLogged;

public class RoutingIOReal implements RoutingIO {
  public static final int KICKER_MOTOR_ID = 12;

  private TalonFX motor = new TalonFX(12);
  private VelocityVoltage velocityOut = new VelocityVoltage(0.0).withEnableFOC(true);

  private StatusSignal<Double> supplyVoltageSignal = motor.getMotorVoltage();
  private StatusSignal<Double> velocity = motor.getRotorVelocity();
  private StatusSignal<Double> currentDraw = motor.getStatorCurrent();

  public RoutingIOReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 1.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kV = 5.0;

    config.CurrentLimits.StatorCurrentLimit = 20.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = 1.0;
    motor.getConfigurator().apply(config);
  }

  public RoutingIOInputsAutoLogged updateInputs() {
    RoutingIOInputsAutoLogged updated = new RoutingIOInputsAutoLogged(); // new values

    updated.currentDrawAmps = currentDraw.getValue();
    updated.velocityRPS = velocity.getValue();
    updated.motorOutputVolts = supplyVoltageSignal.getValue();

    return (updated);
  }

  @Override
  public void setVelocity(double rps) {
    motor.setControl(velocityOut.withVelocity(rps));
  }
}
