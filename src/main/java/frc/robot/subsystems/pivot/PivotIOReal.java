package frc.robot.subsystems.pivot;
import com.ctre.phoenix6.StatusSignal;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

public class PivotIOReal implements PivotIO{

    public static final int PIVOT_MOTOR_ID = 10; 
    public static final double PIVOT_GEAR_RATIO = (27.0 / 1) * (48.0 / 22); //check if this is the correct gear ratio

    private TalonFX pivotMotor = new TalonFX(10);
    private PositionVoltage motorRequest = new PositionVoltage(0.0);
    
    private StatusSignal<Double> supplyVoltageSignal = pivotMotor.getMotorVoltage();
    private StatusSignal<Double> position = pivotMotor.getRotorPosition();
    private StatusSignal<Double> velocity = pivotMotor.getRotorVelocity();
    private StatusSignal<Double> currentDraw = pivotMotor.getStatorCurrent();

    public PivotIOReal (){
        TalonFXConfiguration pivotConfig  = new TalonFXConfiguration();
        pivotConfig.Slot0.kP = 120;
        pivotConfig.Slot0.kD = 0.0;
        pivotConfig.Slot0.kI = 0;

        pivotConfig.Feedback.SensorToMechanismRatio = 58.9;

        pivotConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotMotor.getConfigurator().apply(pivotConfig);
    }

    @Override
    public void setPosition(double degrees) {
        pivotMotor.setControl(motorRequest.withPosition(Units.degreesToRotations(degrees)));    
    }

    @Override
    public void reset(double degrees){ 
        pivotMotor.setPosition((Units.degreesToRotations(degrees)));
    }
   

    @Override
    public PivotIOInputsAutoLogged updateInputs() {
        PivotIOInputsAutoLogged updated = new PivotIOInputsAutoLogged();

        updated.currentDrawAmps = currentDraw.getValue();
        updated.positionRotations = position.getValue();
        updated.velocityRPS = velocity.getValue();
        updated.motorOutputVolts = 12 * supplyVoltageSignal.getValue();

        return(updated);
    }
}