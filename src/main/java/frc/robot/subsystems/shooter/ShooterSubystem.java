package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class ShooterSubystem extends SubsystemBase {
ShooterIO PivotIO;
ShooterIOInputs PivotInputs;
public ShooterSubystem(ShooterIO PivotIO){
this.PivotIO =PivotIO;
PivotInputs = new ShooterIOInputs();



    }
@Override        
public void periodic(){
PivotInputs = PivotIO.updateInputs();
}


}


