package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX pivotMotor = new TalonFX(10); // TODO add right id
  private final TalonFX flywheelLeftMotor = new TalonFX(11); // TODO add right id
  private final TalonFX flywheelRightMotor = new TalonFX(12); // TODO add right id

  private final StatusSignal<Double> pivotVelocity = pivotMotor.getVelocity();
  private final StatusSignal<Double> pivotVoltage = pivotMotor.getMotorVoltage();
  private final StatusSignal<Double> pivotAmps = pivotMotor.getStatorCurrent();
  private final StatusSignal<Double> pivotTempC = pivotMotor.getDeviceTemp();
  private final StatusSignal<Double> pivotRotations = pivotMotor.getPosition();

  private final StatusSignal<Double> flywheelLeftAmps = flywheelLeftMotor.getStatorCurrent();
  private final StatusSignal<Double> flywheelLeftVoltage = flywheelLeftMotor.getMotorVoltage();
  private final StatusSignal<Double> flywheelLeftTempC = flywheelLeftMotor.getDeviceTemp();
  private final StatusSignal<Double> flywheelLeftVelocity = flywheelLeftMotor.getVelocity();

  private final StatusSignal<Double> flywheelRightAmps = flywheelRightMotor.getStatorCurrent();
  private final StatusSignal<Double> flywheelRightVoltage = flywheelRightMotor.getMotorVoltage();
  private final StatusSignal<Double> flywheelRightTempC = flywheelRightMotor.getDeviceTemp();
  private final StatusSignal<Double> flywheelRightVelocity = flywheelRightMotor.getVelocity();

  private final VoltageOut pivotVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut flywheelLeftVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut flywheelRightVoltageOut = new VoltageOut(0.0).withEnableFOC(true);

  private final MotionMagicVoltage pivotMotionMagic =
      new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage flywheelLeftVelocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage flywheelRightVelocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true);

  public ShooterIOReal() {
    var pivotConfig = new TalonFXConfiguration();

    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotConfig.Feedback.SensorToMechanismRatio =
        ShooterSubystem.PIVOT_RATIO; // TODO add in once cad is done

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
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pivotVelocity, pivotVoltage, pivotAmps, pivotTempC, pivotRotations);
    pivotMotor.optimizeBusUtilization();

    var flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    flywheelConfig.Feedback.SensorToMechanismRatio =
        ShooterSubystem.FLYWHEEL_RATIO; // TODO add in once cad is done

    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimit = 40.0;

    flywheelConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    flywheelConfig.Slot0.kG = 0.0; // TODO: Find using sysid or hand tuning
    flywheelConfig.Slot0.kA = 0.0;
    flywheelConfig.Slot0.kS = 0.0;
    flywheelConfig.Slot0.kP = 0.0;
    flywheelConfig.Slot0.kD = 0.0;

    flywheelLeftMotor.getConfigurator().apply(flywheelConfig);
    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelRightMotor.getConfigurator().apply(flywheelConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelLeftVelocity,
        flywheelLeftVoltage,
        flywheelLeftAmps,
        flywheelLeftTempC,
        flywheelRightVelocity,
        flywheelRightVoltage,
        flywheelRightAmps,
        flywheelRightTempC);
    flywheelLeftMotor.optimizeBusUtilization();
    flywheelRightMotor.optimizeBusUtilization();
  }

  @Override
  public ShooterIOInputsAutoLogged updateInputs() {
    BaseStatusSignal.refreshAll(
        pivotRotations,
        pivotVelocity,
        pivotVoltage,
        pivotAmps,
        pivotTempC,
        flywheelLeftVelocity,
        flywheelLeftVoltage,
        flywheelLeftAmps,
        flywheelLeftTempC,
        flywheelRightVelocity,
        flywheelRightVoltage,
        flywheelRightAmps,
        flywheelRightTempC);

    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    inputs.pivotRotation = Rotation2d.fromRotations(pivotRotations.getValue());
    inputs.pivotVelocityRotationsPerSecond = pivotVelocity.getValue();
    inputs.pivotVoltage = pivotVoltage.getValue();
    inputs.pivotAmps = pivotAmps.getValue();
    inputs.pivotTempC = pivotTempC.getValue();

    inputs.flywheelLeftVelocityRotationsPerSecond = flywheelLeftVelocity.getValue();
    inputs.flywheelLeftVoltage = flywheelLeftVoltage.getValue();
    inputs.flywheelLeftAmps = flywheelLeftAmps.getValue();
    inputs.flywheelLeftTempC = flywheelLeftTempC.getValue();

    inputs.flywheelRightVelocityRotationsPerSecond = flywheelRightVelocity.getValue();
    inputs.flywheelRightVoltage = flywheelRightVoltage.getValue();
    inputs.flywheelRightAmps = flywheelRightAmps.getValue();
    inputs.flywheelRightTempC = flywheelRightTempC.getValue();

    return inputs;
  }

  public void setPivotVoltage(final double voltage) {
    pivotMotor.setControl(pivotVoltageOut.withOutput(voltage));
  }

  public void setPivotSetpoint(final Rotation2d rotation) {
    pivotMotor.setControl(pivotMotionMagic.withPosition(rotation.getRotations()));
  }

  public void setFlywheelVoltage(final double left, final double right) {
    flywheelLeftMotor.setControl(flywheelLeftVoltageOut.withOutput(left));
    flywheelRightMotor.setControl(flywheelRightVoltageOut.withOutput(right));
  }

  public void setFlywheelVelocity(final double left, final double right) {
    flywheelLeftMotor.setControl(flywheelLeftVelocityVoltage.withVelocity(left));
    flywheelRightMotor.setControl(flywheelRightVelocityVoltage.withVelocity(right));
  }

  public void resetPivotPostion(final Rotation2d rotation) {
    pivotMotor.setPosition(rotation.getRotations());
  }
}
