package frc.robot.subsystems.routing;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RoutingIOReal implements RoutingIO {
  public static final int KICKER_MOTOR_ID = 12;

  private TalonFX motor = new TalonFX(12);
  private VelocityVoltage velocityOut = new VelocityVoltage(0.0).withEnableFOC(true);
  private VoltageOut voltageOut = new VoltageOut(0.0);

  private StatusSignal<Double> voltage = motor.getMotorVoltage();
  private StatusSignal<Double> velocity = motor.getVelocity();
  private StatusSignal<Double> currentDraw = motor.getStatorCurrent();

  public RoutingIOReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 1.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kV = 0.12;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = 1.0;
    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(50, voltage, velocity, currentDraw);
    motor.optimizeBusUtilization();
  }

  public void updateInputs(RoutingIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(currentDraw, velocity, voltage);
    inputs.currentDrawAmps = currentDraw.getValueAsDouble();
    inputs.velocityRPS = velocity.getValueAsDouble();
    inputs.motorOutputVolts = voltage.getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    motor.setControl(velocityOut.withVelocity(rps));
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }
}
