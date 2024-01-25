package frc.robot.subsystems.routing;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RoutingIOReal implements RoutingIO {
  public static final int KICKER_MOTOR_ID = 12;

  private TalonFX motor = new TalonFX(12);
  private VelocityVoltage velocityOut = new VelocityVoltage(0.0).withEnableFOC(true);

  private StatusSignal<Double> supplyVoltageSignal = motor.getMotorVoltage();
  private StatusSignal<Double> velocity = motor.getVelocity();
  private StatusSignal<Double> currentDraw = motor.getStatorCurrent();

  public RoutingIOReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.1;
    config.Slot0.kD = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kV = 5.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimit = 20.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = 1.0;
    motor.getConfigurator().apply(config);
  }

  public void updateInputs(RoutingIOInputsAutoLogged inputs) {
    inputs.currentDrawAmps = currentDraw.getValueAsDouble();
    inputs.velocityRPS = motor.getVelocity().getValueAsDouble();
    inputs.motorOutputVolts = supplyVoltageSignal.getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    motor.setControl(velocityOut.withVelocity(rps));
  }
}
