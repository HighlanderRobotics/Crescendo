package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

public class KickerIOReal {
    
    private TalonFX kickerMotor = new TalonFX(12);
    private PositionVoltage motorRequest = new PositionVoltage(0.0);
    
    private StatusSignal<Double> supplyVoltageSignal = kickerMotor.getDutyCycle();
    private StatusSignal<Double> position = kickerMotor.getRotorPosition();
    private StatusSignal<Double> velocity = kickerMotor.getRotorVelocity();
    private StatusSignal<Double> currentDraw = kickerMotor.getStatorCurrent();

    public KickerIOReal (){
        TalonFXConfiguration kickerConfig  = new TalonFXConfiguration();
        kickerConfig.Slot0.kP = 1;
        kickerConfig.Slot0.kD = 0.01;
        kickerConfig.Slot0.kI = 0;
    }

    public void setPosition(double degrees) {
        kickerMotor.setControl(motorRequest.withPosition(Units.degreesToRotations(degrees)));    
    }

    public void reset(double degrees){ 
        kickerMotor.setPosition((Units.degreesToRotations(degrees)));
    }

    public KickerIOInputsAutoLogged updateInputs() {
        KickerIOInputsAutoLogged current = new KickerIOInputsAutoLogged();

        current.currentDrawAmps = currentDraw.refresh().getValue();
        current.positionRotations = position.refresh().getValue();
        current.velocityRPM = velocity.refresh().getValue();
        current.motorOutputVolts = 12 * supplyVoltageSignal.getValue();

        return(current);
    }
}