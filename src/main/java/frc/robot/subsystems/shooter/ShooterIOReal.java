package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOReal implements ShooterIO {

  public static final int SHOOTER_MOTOR_ID = 27;
  public static final double SHOOTER_GEAR_RATIO = 1;

  private TalonFX shooterMotor = new TalonFX(27);
  private VelocityVoltage shooterMotorVelocity = new VelocityVoltage(0);
  private VoltageOut voltageOut = new VoltageOut(0.0);

  private StatusSignal<Double> supplyVoltageSignal = shooterMotor.getMotorVoltage();
  private StatusSignal<Double> velocity = shooterMotor.getRotorVelocity();
  private StatusSignal<Double> currentDraw = shooterMotor.getStatorCurrent();

  public ShooterIOReal() {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = 0;
    shooterConfig.Slot0.kD = 0;
    shooterConfig.Slot0.kI = 0;
    shooterConfig.Slot0.kV = 0;
  }

  public void setVelocity(double velocity) {
    shooterMotor.setControl(voltageOut.withOutput(velocity));
  }

  public ShooterIOInputsAutoLogged updateInputs() {
    ShooterIOInputsAutoLogged updated = new ShooterIOInputsAutoLogged();

    updated.currentDrawAmps = currentDraw.getValue();
    updated.velocityRPS = velocity.getValue();
    updated.motorOutputVolts = 12 * supplyVoltageSignal.getValue();

    return (updated);
  }
}
