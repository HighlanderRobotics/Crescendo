package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

public class Shooterreal implements ShooterIO{

private final TalonFX motor = new TalonFX(0);
public ShooterIOInputs updateInputs() {
        
    
  ShooterIOInputs inputs = new ShooterIOInputs();
  inputs.PivotAmps = motor.getVelocity().getValue();
  inputs.PivotTempc = motor.getDeviceTemp().getValue();
  inputs.PivotMeters = motor.getRotorPosition().getValue();
  inputs.PivotVoltage = motor.getSupplyVoltage().getValue();

  inputs.FlywheelAmp = motor.getVelocity().getValue();
  inputs.FlywheelTempc = motor.getDeviceTemp().getValue();
  inputs.FlywheelVoltage = motor.getSupplyVoltage().getValue();
  return inputs;
}

}







