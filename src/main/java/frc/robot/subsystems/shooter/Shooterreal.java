package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class Shooterreal implements ShooterIO {
  private final TalonFX motor = new TalonFX(0);
  private double pivotAppliedVolts;

  @Override
  public ShooterIOInputs updateInputs() {
    ShooterIOInputs inputs = new ShooterIOInputs();
    inputs.PivotAmps = motor.getSupplyCurrent().getValue();
    inputs.PivotTempc = motor.getDeviceTemp().getValue();
    inputs.PivotMeters = motor.getRotorPosition().getValue();
    inputs.PivotVoltage = motor.getSupplyVoltage().getValue();
    inputs.PivotVelocity = motor.getVelocity().getValue();
    inputs.FlywheelAmp = motor.getSupplyCurrent().getValue();
    inputs.FlywheelTempc = motor.getDeviceTemp().getValue();
    inputs.FlywheelVoltage = motor.getSupplyVoltage().getValue();
    inputs.FlywheelVelocity = motor.getVelocity().getValue();
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    return inputs;
  }

  public void setFlywheelVelocity(final Rotation2d rotation) {
    setFlywheelVoltage(pivotAppliedVolts = MathUtil.clamp(pivotAppliedVolts, -12.0, 12.0));
    motor.setVoltage(pivotAppliedVolts);
  }

  private final PIDController pivotController = new PIDController(100.0, 0.0, 0.0);
}
