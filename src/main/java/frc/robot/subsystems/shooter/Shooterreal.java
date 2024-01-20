package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;



public class Shooterreal implements ShooterIO{

private final TalonFX motor = new TalonFX(0);
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
  return inputs;
}
public double getFlywheelVoltage() {
  return updateInputs().FlywheelVoltage;
}

public double getFlywheelTempc() {
  return updateInputs().FlywheelTempc;
}

public double getFlywheelAmp() {
  return updateInputs().FlywheelAmp;
}

public double getPivotVoltage() {
  return updateInputs().PivotVoltage;
}

public double getPivotMeters() {
  return updateInputs().PivotMeters;
}

public double getPivotTempc() {
  return updateInputs().PivotTempc;
}

public double getPivotAmps() {
  return updateInputs().PivotAmps;
}




}







