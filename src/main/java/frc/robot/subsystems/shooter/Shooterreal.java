package frc.robot.subsystems.shooter;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
public class Shooterreal implements ShooterIO{
private final TalonFX motor = new TalonFX(0);
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
  inputs.driveVelocityMetersPerSec = turnSim.getAngularVelocityRadPerSec();
  inputs.drivePositionMeters = turnSim.getAngularPositionRad();
  inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
  inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
  return inputs;
}

private DCMotorSim turnSim =
  new DCMotorSim(DCMotor.getKrakenX60Foc(1), 0, 0);
  
  public Rotation2d getAngle() {
   return updateInputs().turnPosition;
  }

public void setTurnSetpoint(final Rotation2d rotation) {
    setTurnVoltage(
        turnController.calculate(turnSim.getAngularPositionRotations(), rotation.getRotations()));
  }
private final PIDController turnController = new PIDController(100.0, 0.0, 0.0);


}


 
  










