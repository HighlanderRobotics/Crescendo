package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ShooterIOReal {
    
    private TalonFX bottomShooterMotor = new TalonFX(Constants.BOTTOM_SHOOTER_MOTOR_ID);
    private VelocityVoltage bottomShooterMotorVelocity = new VelocityVoltage(0);


    private StatusSignal<Double> supplyVoltageSignal = bottomShooterMotor.getDutyCycle();
    private StatusSignal<Double> velocity = bottomShooterMotor.getRotorVelocity();
    private StatusSignal<Double> currentDraw = bottomShooterMotor.getStatorCurrent();

    public ShooterIOReal(){
        TalonFXConfiguration pivotConfig  = new TalonFXConfiguration();
        pivotConfig.Slot0.kP = 1;
        pivotConfig.Slot0.kD = 0.01;
        pivotConfig.Slot0.kI = 0;
    }

    public void setVelocity(double velocity) {
        bottomShooterMotor.setControl(bottomShooterMotorVelocity.withVelocity(velocity));
    }


    public ShooterIOInputsAutoLogged updateInputs() {
        ShooterIOInputsAutoLogged current = new ShooterIOInputsAutoLogged();

        current.currentDrawAmps = currentDraw.refresh().getValue();
        current.velocityRPM = velocity.refresh().getValue() / Constants.BOTTOM_SHOOTER_GEAR_RATIO;
        current.motorOutputVolts = 12 * supplyVoltageSignal.getValue();

        return(current);
    }
}