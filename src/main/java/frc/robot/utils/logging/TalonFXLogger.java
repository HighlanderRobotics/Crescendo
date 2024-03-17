package frc.robot.utils.logging;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/**
 * Helper class for logging TalonFX data with advantagescope. This logs talonFX data using either
 * rotations or meters, depending on how the mechanism is configured.
 */
public class TalonFXLogger {
  // Use generics for units
  public static class TalonFXLog implements StructSerializable {
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double position = 0.0;
    public double velocity = 0.0;

    public TalonFXLog(
        double appliedVolts,
        double statorCurrentAmps,
        double supplyCurrentAmps,
        double temperatureCelsius,
        double position,
        double velocityRotationsPerSecond) {
      this.appliedVolts = appliedVolts;
      this.statorCurrentAmps = statorCurrentAmps;
      this.supplyCurrentAmps = supplyCurrentAmps;
      this.temperatureCelsius = temperatureCelsius;
      this.position = position;
      this.velocity = velocityRotationsPerSecond;
    }

    public static TalonFXLogStruct struct = new TalonFXLogStruct();

    private static class TalonFXLogStruct implements Struct<TalonFXLog> {

      @Override
      public Class<TalonFXLog> getTypeClass() {
        return TalonFXLog.class;
      }

      @Override
      public String getTypeString() {
        return "struct:TalonFXLog";
      }

      @Override
      public int getSize() {
        return kSizeDouble * 6;
      }

      @Override
      public String getSchema() {
        return "double voltage;double statorAmps;double supplyAmps;double temp;double position;double velocity";
      }

      @Override
      public TalonFXLog unpack(ByteBuffer bb) {
        double voltage = bb.getDouble();
        double statorAmps = bb.getDouble();
        double supplyAmps = bb.getDouble();
        double temp = bb.getDouble();
        double rotation = bb.getDouble();
        double velocity = bb.getDouble();
        return new TalonFXLog(voltage, statorAmps, supplyAmps, temp, rotation, velocity);
      }

      @Override
      public void pack(ByteBuffer bb, TalonFXLog value) {
        bb.putDouble(value.appliedVolts);
        bb.putDouble(value.statorCurrentAmps);
        bb.putDouble(value.supplyCurrentAmps);
        bb.putDouble(value.temperatureCelsius);
        bb.putDouble(value.position);
        bb.putDouble(value.velocity);
      }
    }
  }

  public final TalonFXLog log = new TalonFXLog(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  private final StatusSignal<Double> voltageSignal;
  private final StatusSignal<Double> statorCurrentSignal;
  private final StatusSignal<Double> supplyCurrentSignal;
  private final StatusSignal<Double> temperatureSignal;
  private final StatusSignal<Double> rotationSignal;
  private final StatusSignal<Double> velocitySignal;

  public TalonFXLogger(TalonFX talon, double updateFrequencyHz, boolean shouldOptimizeBusUsage) {
    voltageSignal = talon.getMotorVoltage();
    statorCurrentSignal = talon.getStatorCurrent();
    supplyCurrentSignal = talon.getSupplyCurrent();
    temperatureSignal = talon.getDeviceTemp();
    rotationSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        updateFrequencyHz,
        voltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        temperatureSignal,
        rotationSignal,
        velocitySignal);
    if (shouldOptimizeBusUsage) talon.optimizeBusUtilization();
  }

  public TalonFXLogger(TalonFX talon) {
    this(talon, 50.0, true);
  }

  public TalonFXLog update() {
    BaseStatusSignal.refreshAll(
        voltageSignal, statorCurrentSignal, supplyCurrentSignal, temperatureSignal);
    log.appliedVolts = voltageSignal.getValueAsDouble();
    log.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
    log.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    log.temperatureCelsius = temperatureSignal.getValueAsDouble();
    log.position = rotationSignal.getValueAsDouble();
    log.velocity = rotationSignal.getValueAsDouble();

    return log;
  }
}
