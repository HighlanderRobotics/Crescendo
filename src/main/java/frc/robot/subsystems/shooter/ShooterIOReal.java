package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX pivotMotor = new TalonFX(10); // TODO add right id
  private final TalonFX flywheelMotor = new TalonFX(11); // TODO add right id

  private final StatusSignal<Double> pivotVelocity = pivotMotor.getVelocity();
  private final StatusSignal<Double> pivotVoltage = pivotMotor.getMotorVoltage();
  private final StatusSignal<Double> pivotAmps = pivotMotor.getStatorCurrent();
  private final StatusSignal<Double> pivotTempC = pivotMotor.getDeviceTemp();
  private final StatusSignal<Double> pivotRotations = pivotMotor.getPosition();

  private final StatusSignal<Double> flywheelAmps = flywheelMotor.getStatorCurrent();
  private final StatusSignal<Double> flywheelvoltage = flywheelMotor.getMotorVoltage();
  private final StatusSignal<Double> flywheelTempC = flywheelMotor.getDeviceTemp();
  private final StatusSignal<Double> flywheelVelocity = flywheelMotor.getVelocity();

  private final VoltageOut pivotVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut flywheelVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage pivotMotionMagic =
      new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage flywheelVelocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true);

  public ShooterIOReal() {
    var pivotConfig = new TalonFXConfiguration();

    pivotConfig.Feedback.SensorToMechanismRatio = 35.0 / 1.0; // TODO add in once cad is done

    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40.0;

    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.Slot0.kG = 0.0; // TODO: Find using sysid or hand tuning
    pivotConfig.Slot0.kA = 0.0;
    pivotConfig.Slot0.kS = 0.0;
    pivotConfig.Slot0.kP = 0.0;
    pivotConfig.Slot0.kD = 0.0;

    pivotConfig.MotionMagic.MotionMagicAcceleration = 1.0;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;

    pivotMotor.getConfigurator().apply(pivotConfig);

    var flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Feedback.SensorToMechanismRatio = 18.0 / 24.0; // TODO add in once cad is done

    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimit = 40.0;

    flywheelConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    flywheelConfig.Slot0.kG = 0.0; // TODO: Find using sysid or hand tuning
    flywheelConfig.Slot0.kA = 0.0;
    flywheelConfig.Slot0.kS = 0.0;
    flywheelConfig.Slot0.kP = 0.0;
    flywheelConfig.Slot0.kD = 0.0;

    flywheelMotor.getConfigurator().apply(pivotConfig);
  }

  @Override
  public ShooterIOInputsAutoLogged updateInputs() {
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    inputs.pivotRotation = Rotation2d.fromRotations(pivotRotations.getValue());
    inputs.pivotVelocityRotationsPerSecond = pivotVelocity.getValue();
    inputs.pivotVoltage = pivotVoltage.getValue();
    inputs.pivotAmps = pivotAmps.getValue();
    inputs.pivotTempC = pivotTempC.getValue();

    inputs.flywheelAmps = flywheelAmps.getValue();
    inputs.flywheelVoltage = flywheelvoltage.getValue();
    inputs.flywheelTempC = flywheelTempC.getValue();
    inputs.flywheelVelocity = flywheelVelocity.getValue();

    return inputs;
  }

  public void setPivotVoltage(final double voltage) {
    pivotMotor.setControl(pivotVoltageOut.withOutput(voltage));
  }

  public void setPivotSetpoint(final Rotation2d rotation) {
    pivotMotor.setControl(pivotMotionMagic.withPosition(rotation.getRotations()));
  }

  public void setFlywheelVoltage(final double voltage) {
    flywheelMotor.setControl(flywheelVoltageOut.withOutput(voltage));
  }

  public void setFlywheelVelocity(final double rps) {
    flywheelMotor.setControl(flywheelVelocityVoltage.withVelocity(rps));
  }

  public void resetPivotPostion(final Rotation2d rotation) {
    pivotMotor.setPosition(rotation.getRotations());
  }
}
