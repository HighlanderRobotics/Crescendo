package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    
    ShooterIO io;
    ShooterIOInputsAutoLogged inputs;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        inputs =  new ShooterIOInputsAutoLogged();
    }

    public RunCommand run(double velocity) {
        return new RunCommand(() -> {
            io.setVelocity(velocity);
        }, this);
    }  

    @Override
    public void periodic() {
        inputs = io.updateInputs();
        Logger.processInputs("Shooter", inputs);
    }
}