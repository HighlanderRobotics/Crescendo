package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOReal implements ShooterIO {

  public static final int SHOOTER_MOTOR_ID = 27;
  public static final double SHOOTER_GEAR_RATIO = 1;

  private TalonFX shooterMotor = new TalonFX(27);
  private VelocityVoltage shooterMotorVelocity = new VelocityVoltage(0).withEnableFOC(true);
  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  private StatusSignal<Double> supplyVoltageSignal;
  private StatusSignal<Double> velocity;
  private StatusSignal<Double> currentDraw;

  public ShooterIOReal() {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = 0.1;
    shooterConfig.Slot0.kD = 0;
    shooterConfig.Slot0.kI = 0;
    shooterConfig.Slot0.kV = 5.0;

    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    shooterConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterMotor.getConfigurator().apply(shooterConfig);

    supplyVoltageSignal = shooterMotor.getMotorVoltage();
    velocity = shooterMotor.getVelocity();
    currentDraw = shooterMotor.getStatorCurrent();
  }

  public void setVoltage(double voltage) {
    shooterMotor.setControl(voltageOut.withOutput(voltage));
  }

  public void setVelocity(double rps) {
    shooterMotor.setControl(shooterMotorVelocity.withVelocity(rps));
  }

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    inputs.currentDrawAmps = currentDraw.getValueAsDouble();
    inputs.velocityRPS = shooterMotor.getVelocity().getValueAsDouble();
    inputs.motorOutputVolts = supplyVoltageSignal.getValueAsDouble();
    inputs.temperatureCelsius = 20.0;
  }
}
