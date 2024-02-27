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
  private final TalonFX pivotMotor = new TalonFX(10);
  private final TalonFX flywheelLeftMotor = new TalonFX(11);
  private final TalonFX flywheelRightMotor = new TalonFX(12);

  private final StatusSignal<Double> pivotVelocity = pivotMotor.getVelocity();
  private final StatusSignal<Double> pivotVoltage = pivotMotor.getMotorVoltage();
  private final StatusSignal<Double> pivotStatorCurrentAmps = pivotMotor.getStatorCurrent();
  private final StatusSignal<Double> pivotSupplyCurrentAmps = pivotMotor.getSupplyCurrent();
  private final StatusSignal<Double> pivotTempC = pivotMotor.getDeviceTemp();
  private final StatusSignal<Double> pivotRotations = pivotMotor.getPosition();

  private final StatusSignal<Double> flywheelLeftStatorCurrentAmps = flywheelLeftMotor.getStatorCurrent();
  private final StatusSignal<Double> flywheelLeftSupplyCurrentAmps = flywheelLeftMotor.getSupplyCurrent();
  private final StatusSignal<Double> flywheelLeftVoltage = flywheelLeftMotor.getMotorVoltage();
  private final StatusSignal<Double> flywheelLeftTempC = flywheelLeftMotor.getDeviceTemp();
  private final StatusSignal<Double> flywheelLeftVelocity = flywheelLeftMotor.getVelocity();

  private final StatusSignal<Double> flywheelRightStatorCurrentAmps = flywheelRightMotor.getStatorCurrent();
  private final StatusSignal<Double> flywheelRightSupplyCurrentAmps = flywheelRightMotor.getSupplyCurrent();
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
    pivotMotor.setPosition(
        ShooterSubystem.PIVOT_MIN_ANGLE.getRotations()); // Assume we boot at hard stop
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pivotVelocity, pivotVoltage, pivotStatorCurrentAmps, pivotSupplyCurrentAmps, pivotTempC, pivotRotations);
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
        flywheelLeftStatorCurrentAmps,
        flywheelLeftSupplyCurrentAmps,
        flywheelLeftTempC,
        flywheelRightVelocity,
        flywheelRightVoltage,
        flywheelRightStatorCurrentAmps,
        flywheelRightSupplyCurrentAmps,
        flywheelRightTempC);
    flywheelLeftMotor.optimizeBusUtilization();
    flywheelRightMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        pivotRotations,
        pivotVelocity,
        pivotVoltage,
        pivotStatorCurrentAmps,
        pivotSupplyCurrentAmps,
        pivotTempC,
        flywheelLeftVelocity,
        flywheelLeftVoltage,
        flywheelLeftStatorCurrentAmps,
        flywheelLeftSupplyCurrentAmps,
        flywheelLeftTempC,
        flywheelRightVelocity,
        flywheelRightVoltage,
        flywheelRightStatorCurrentAmps,
        flywheelRightSupplyCurrentAmps,
        flywheelRightTempC);

    inputs.pivotRotation = Rotation2d.fromRotations(pivotRotations.getValueAsDouble());
    inputs.pivotVelocityRotationsPerSecond = pivotVelocity.getValueAsDouble();
    inputs.pivotVoltage = pivotVoltage.getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivotStatorCurrentAmps.getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivotSupplyCurrentAmps.getValueAsDouble();
    inputs.pivotTempC = pivotTempC.getValueAsDouble();

    inputs.flywheelLeftVelocityRotationsPerSecond = flywheelLeftVelocity.getValueAsDouble();
    inputs.flywheelLeftVoltage = flywheelLeftVoltage.getValueAsDouble();
    inputs.flywheelLeftStatorCurrentAmps = flywheelLeftStatorCurrentAmps.getValueAsDouble();
    inputs.flywheelLeftSupplyCurrentAmps = flywheelLeftSupplyCurrentAmps.getValueAsDouble();
    inputs.flywheelLeftTempC = flywheelLeftTempC.getValueAsDouble();

    inputs.flywheelRightVelocityRotationsPerSecond = flywheelRightVelocity.getValueAsDouble();
    inputs.flywheelRightVoltage = flywheelRightVoltage.getValueAsDouble();
    inputs.flywheelRightStatorCurrentAmps = flywheelRightStatorCurrentAmps.getValueAsDouble();
    inputs.flywheelRightSupplyCurrentAmps = flywheelRightSupplyCurrentAmps.getValueAsDouble();
    inputs.flywheelRightTempC = flywheelRightTempC.getValueAsDouble();
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
