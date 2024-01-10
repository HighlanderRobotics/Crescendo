package frc.robot.subsystems.pivot;
import com.ctre.phoenix6.StatusSignal;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class PivotIOReal implements PivotIO{

    private TalonFX pivotMotor = new TalonFX(Constants.PIVOT_MOTOR_ID);
    private PositionVoltage motorRequest = new PositionVoltage(Constants.PIVOT_MOTOR_ID);
    
    private StatusSignal<Double> supplyVoltageSignal = pivotMotor.getDutyCycle();
    private StatusSignal<Double> position = pivotMotor.getRotorPosition();
    private StatusSignal<Double> velocity = pivotMotor.getRotorVelocity();
    private StatusSignal<Double> currentDraw = pivotMotor.getStatorCurrent();


    public PivotIOReal (){
        TalonFXConfiguration pivotConfig  = new TalonFXConfiguration();
        pivotConfig.Slot0.kP = 1;
        pivotConfig.Slot0.kD = 0.01;
        pivotConfig.Slot0.kI = 0;
    }

    @Override
    public void setPosition(double degrees) {
        pivotMotor.setControl(motorRequest.withPosition(Units.degreesToRotations(degrees)*Constants.PIVOT_GEAR_RATIO));    
    }

    @Override
    public void reset(double degrees){ 
        pivotMotor.setPosition((Units.degreesToRotations(degrees))*Constants.PIVOT_GEAR_RATIO);
    }
   

    @Override
    public PivotIOInputsAutoLogged updateInputs() {
        PivotIOInputsAutoLogged current = new PivotIOInputsAutoLogged();

        current.currentDrawAmps = currentDraw.refresh().getValue();
        current.positionRotations = position.refresh().getValue() / Constants.PIVOT_GEAR_RATIO;
        current.velocityRPM = velocity.refresh().getValue() / Constants.PIVOT_GEAR_RATIO;
        current.motorOutputVolts = 12 * supplyVoltageSignal.getValue();

        return(current);
    }
}