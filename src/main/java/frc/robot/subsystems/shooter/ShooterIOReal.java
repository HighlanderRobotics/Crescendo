package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOReal implements ShooterIO {

  public static final int SHOOTER_MOTOR_ID = 27;
  public static final double SHOOTER_GEAR_RATIO = 1;

  private TalonFX motor = new TalonFX(27);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withEnableFOC(true);
  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  private StatusSignal<Double> voltage = motor.getMotorVoltage();
  private StatusSignal<Double> velocity = motor.getVelocity();
  private StatusSignal<Double> currentDraw = motor.getStatorCurrent();

  public ShooterIOReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.1;
    config.Slot0.kD = 0;
    config.Slot0.kI = 0;
    config.Slot0.kV = 5.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(config);

    voltage = motor.getMotorVoltage();
    velocity = motor.getVelocity();
    currentDraw = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50, voltage, velocity, currentDraw);
    motor.optimizeBusUtilization();
  }

  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  public void setVelocity(double rps) {
    motor.setControl(velocityVoltage.withVelocity(rps));
  }

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    
    inputs.currentDrawAmps = currentDraw.getValueAsDouble();
    inputs.velocityRPS = velocity.getValueAsDouble();
    inputs.motorOutputVolts = voltage.getValueAsDouble();
    inputs.temperatureCelsius = 20.0;
  }
}
