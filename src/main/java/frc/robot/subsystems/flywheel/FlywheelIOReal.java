package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class FlywheelIOReal implements FlywheelIO {
  private final TalonFX motor;

  private final StatusSignal<Double> amps;
  private final StatusSignal<Double> voltage;
  private final StatusSignal<Double> temp;
  private final StatusSignal<Double> vel;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

  public FlywheelIOReal(int id, InvertedValue invert) {
    motor = new TalonFX(id, "canivore");

    amps = motor.getStatorCurrent();
    voltage = motor.getMotorVoltage();
    temp = motor.getDeviceTemp();
    vel = motor.getVelocity();

    var flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.MotorOutput.Inverted = invert;

    flywheelConfig.Feedback.SensorToMechanismRatio = FlywheelSubsystem.FLYWHEEL_RATIO;

    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

    flywheelConfig.Slot0.kA = 0.0051316;
    flywheelConfig.Slot0.kV = 0.095;
    flywheelConfig.Slot0.kS = 0.3;
    flywheelConfig.Slot0.kP = 0.1;
    flywheelConfig.Slot0.kD = 0.0;

    motor.getConfigurator().apply(flywheelConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, vel, voltage, amps, temp);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(vel, voltage, amps, temp);
    inputs.velocityRotationsPerSecond = vel.getValue();
    inputs.voltage = voltage.getValue();
    inputs.amps = amps.getValue();
    inputs.tempC = temp.getValue();
  }

  @Override
  public void setVoltage(final double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setVelocity(final double rps) {
    motor.setControl(velocityVoltage.withVelocity(rps));
  }

  @Override
  public void setCurrentLimit(final double stator, final double supply) {
    motor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(stator)
                .withSupplyCurrentLimit(supply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));
  }
}
