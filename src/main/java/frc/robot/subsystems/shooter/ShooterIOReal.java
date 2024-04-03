package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.logging.TalonFXLogger;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX pivotMotor = new TalonFX(10, "canivore");
  private final TalonFX flywheelLeftMotor = new TalonFX(11, "canivore");
  private final TalonFX flywheelRightMotor = new TalonFX(12, "canivore");

  private final TalonFXLogger pivotLogger = new TalonFXLogger(pivotMotor);

  private final TalonFXLogger flywheelLeftLogger = new TalonFXLogger(flywheelLeftMotor);

  private final TalonFXLogger flywheelRightLogger = new TalonFXLogger(flywheelRightMotor);

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

    pivotConfig.Feedback.SensorToMechanismRatio = ShooterSubsystem.PIVOT_RATIO;

    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40.0;

    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.Slot0.kG = 0.5;
    pivotConfig.Slot0.kV = 7.2;
    pivotConfig.Slot0.kA = 0.1;
    pivotConfig.Slot0.kS = 0.0;
    pivotConfig.Slot0.kP = 400.0;
    pivotConfig.Slot0.kD = 0.0;

    pivotConfig.MotionMagic.MotionMagicAcceleration = 1.0;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;

    pivotMotor.getConfigurator().apply(pivotConfig);
    pivotMotor.setPosition(
        ShooterSubsystem.PIVOT_MIN_ANGLE.getRotations()); // Assume we boot at hard stop
    pivotMotor.optimizeBusUtilization();

    var flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    flywheelConfig.Feedback.SensorToMechanismRatio =
        ShooterSubsystem.FLYWHEEL_RATIO; // TODO add in once cad is done

    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

    flywheelConfig.Slot0.kA = 0.0051316;
    flywheelConfig.Slot0.kV = 0.095;
    flywheelConfig.Slot0.kS = 0.3;
    flywheelConfig.Slot0.kP = 0.1;
    flywheelConfig.Slot0.kD = 0.0;

    flywheelLeftMotor.getConfigurator().apply(flywheelConfig);
    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelRightMotor.getConfigurator().apply(flywheelConfig);
    flywheelLeftMotor.optimizeBusUtilization();
    flywheelRightMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    inputs.pivot = pivotLogger.update();

    inputs.leftFlywheel = flywheelLeftLogger.update();

    inputs.rightFlywheel = flywheelRightLogger.update();
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

  public void resetPivotPosition(final Rotation2d rotation) {
    pivotMotor.setPosition(rotation.getRotations());
  }
}
