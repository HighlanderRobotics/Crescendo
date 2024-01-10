package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

public class KickerIOReal implements KickerIO {
    
    private TalonFX kickerMotor = new TalonFX(12);
    private PositionVoltage motorRequest = new PositionVoltage(0.0);
    
    private StatusSignal<Double> supplyVoltageSignal = kickerMotor.getDutyCycle();
    private StatusSignal<Double> position = kickerMotor.getRotorPosition();
    private StatusSignal<Double> velocity = kickerMotor.getRotorVelocity();
    private StatusSignal<Double> currentDraw = kickerMotor.getStatorCurrent();

    public KickerIOReal (){
        TalonFXConfiguration kickerConfig  = new TalonFXConfiguration();
        kickerConfig.Slot0.kP = 10.0;
        kickerConfig.Slot0.kD = 0.0;
        kickerConfig.Slot0.kI = 0.0;
        kickerConfig.CurrentLimits.StatorCurrentLimit = 20.0;
        kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kickerConfig.Feedback.SensorToMechanismRatio = 125.0;
        kickerMotor.getConfigurator().apply(kickerConfig);
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