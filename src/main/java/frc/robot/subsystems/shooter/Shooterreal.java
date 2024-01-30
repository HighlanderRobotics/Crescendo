package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class Shooterreal implements ShooterIO {
  private final TalonFX motor = new TalonFX(0);
  private double pivotAppliedVolts;
  public void setPivotAngle(double targetAngle) {}
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
    double volts = calculateVoltsFromRotation(rotation);
    double pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    
    setFlywheelVoltage(pivotAppliedVolts);
    motor.setVoltage(pivotAppliedVolts);
}

private double calculateVoltsFromRotation(Rotation2d rotation) {
    inputs.FlywheelVoltage = motor.getSupplyVoltage().getValue();
    return calculatedVolts;
}


private void setFlywheelVoltage(double volts) {
    motor.setVoltage(volts);
}

}



  private final PIDController pivotController = new PIDController(100.0, 0.0, 0.0);

