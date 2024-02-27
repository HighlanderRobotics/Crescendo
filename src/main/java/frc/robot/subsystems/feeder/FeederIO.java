// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double feederVelocityRotationsPerSec = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederStatorCurrentAmps = 0.0;
    public double feederSupplyCurrentAmps = 0.0;
    public double feederTempC = 0.0;

    public boolean firstBeambreak = false;
    public boolean lastBeambreak = false;
  }

  public void updateInputs(final FeederIOInputsAutoLogged inputs);

  public void setVoltage(final double volts);
}
