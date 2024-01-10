package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOReal implements ShooterIO {
    
    private TalonFX bottomShooterMotor = new TalonFX(27);
    private VelocityVoltage bottomShooterMotorVelocity = new VelocityVoltage(0);
    private VoltageOut voltageOut = new VoltageOut(0.0);

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
        bottomShooterMotor.setControl(voltageOut.withOutput(velocity));
    }

    public ShooterIOInputsAutoLogged updateInputs() {
        ShooterIOInputsAutoLogged current = new ShooterIOInputsAutoLogged();

        current.currentDrawAmps = currentDraw.refresh().getValue();
        current.velocityRPM = velocity.refresh().getValue();
        current.motorOutputVolts = 12 * supplyVoltageSignal.getValue();

        return(current);
    }
}