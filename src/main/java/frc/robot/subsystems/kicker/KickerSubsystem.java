package frc.robot.subsystems.kicker;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class KickerSubsystem extends SubsystemBase {
    KickerIO io;
    KickerIOInputsAutoLogged inputs;


    public KickerSubsystem(KickerIO io) {
        this.io = io;
        inputs =  new KickerIOInputsAutoLogged();

        SmartDashboard.putData("reset Kicker value", new InstantCommand(
            ()->{io.reset(0);}
            ));
    }

    public InstantCommand reset (){
       return new InstantCommand(
            ()->{io.reset(0);}
            );
    }

    public RunCommand run(double degrees) {
        return new RunCommand(() -> {
            io.setPosition(degrees);
        }, this);
    }   

    @Override
    public void periodic() {
        inputs = io.updateInputs();
        Logger.processInputs("Kicker", inputs);
    }
}