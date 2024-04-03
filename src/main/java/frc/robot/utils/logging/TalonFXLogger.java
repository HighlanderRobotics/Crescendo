package frc.robot.utils.logging;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
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

    // StatusCode error value. Currently just checks one signal.
    public StatusCode errorCode = StatusCode.StatusCodeNotInitialized;

    // Faults
    public boolean licenseFault = false;

    public TalonFXLog(
        double appliedVolts,
        double statorCurrentAmps,
        double supplyCurrentAmps,
        double temperatureCelsius,
        double position,
        double velocityRotationsPerSecond,
        StatusCode errorCode,
        boolean licenseFault) {
      this.appliedVolts = appliedVolts;
      this.statorCurrentAmps = statorCurrentAmps;
      this.supplyCurrentAmps = supplyCurrentAmps;
      this.temperatureCelsius = temperatureCelsius;
      this.position = position;
      this.velocity = velocityRotationsPerSecond;

      this.errorCode = errorCode;

      this.licenseFault = licenseFault;
    }

    public TalonFXLog(
        double appliedVolts,
        double statorCurrentAmps,
        double supplyCurrentAmps,
        double temperatureCelsius,
        double position,
        double velocityRotationsPerSecond,
        StatusCode errorCode) {
      this(
          appliedVolts,
          statorCurrentAmps,
          supplyCurrentAmps,
          temperatureCelsius,
          position,
          velocityRotationsPerSecond,
          errorCode,
          false);
    }

    public TalonFXLog(
        double appliedVolts,
        double statorCurrentAmps,
        double supplyCurrentAmps,
        double temperatureCelsius,
        double position,
        double velocityRotationsPerSecond) {
      this(
          appliedVolts,
          statorCurrentAmps,
          supplyCurrentAmps,
          temperatureCelsius,
          position,
          velocityRotationsPerSecond,
          StatusCode.StatusCodeNotInitialized,
          false);
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
        return kSizeDouble * 6 + kSizeInt32 + kSizeBool * 1;
      }

      @Override
      public String getSchema() {
        return "double voltage;double statorAmps;double supplyAmps;double temp;double position;double velocity;int errorCode;boolean licenseFault";
      }

      @Override
      public TalonFXLog unpack(ByteBuffer bb) {
        double voltage = bb.getDouble();
        double statorAmps = bb.getDouble();
        double supplyAmps = bb.getDouble();
        double temp = bb.getDouble();
        double rotation = bb.getDouble();
        double velocity = bb.getDouble();
        var errorCode = StatusCode.valueOf(bb.getInt());
        boolean licenseFault = bb.get() != 0;
        return new TalonFXLog(
            voltage, statorAmps, supplyAmps, temp, rotation, velocity, errorCode, licenseFault);
      }

      @Override
      public void pack(ByteBuffer bb, TalonFXLog value) {
        bb.putDouble(value.appliedVolts);
        bb.putDouble(value.statorCurrentAmps);
        bb.putDouble(value.supplyCurrentAmps);
        bb.putDouble(value.temperatureCelsius);
        bb.putDouble(value.position);
        bb.putDouble(value.velocity);
        bb.put(value.licenseFault ? (byte) 0 : (byte) 1);
      }
    }
  }

  public final TalonFXLog log = new TalonFXLog(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  private final StatusSignal<Double> voltageSignal;
  private final StatusSignal<Double> statorCurrentSignal;
  private final StatusSignal<Double> supplyCurrentSignal;
  private final StatusSignal<Double> temperatureSignal;
  private final StatusSignal<Double> positionSignal;
  private final StatusSignal<Double> velocitySignal;

  private final StatusSignal<Boolean> licenseFaultSignal;

  public TalonFXLogger(
      TalonFX talon,
      double updateFrequencyHz,
      double faultFrequencyHz,
      boolean shouldOptimizeBusUsage) {
    voltageSignal = talon.getMotorVoltage();
    statorCurrentSignal = talon.getStatorCurrent();
    supplyCurrentSignal = talon.getSupplyCurrent();
    temperatureSignal = talon.getDeviceTemp();
    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();

    licenseFaultSignal = talon.getFault_UnlicensedFeatureInUse();

    BaseStatusSignal.setUpdateFrequencyForAll(
        updateFrequencyHz,
        voltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        temperatureSignal,
        positionSignal,
        velocitySignal);
    BaseStatusSignal.setUpdateFrequencyForAll(faultFrequencyHz, licenseFaultSignal);
    if (shouldOptimizeBusUsage) talon.optimizeBusUtilization();
  }

  public TalonFXLogger(TalonFX talon) {
    this(talon, 50.0, 1.0, true);
  }

  public TalonFXLog update() {
    BaseStatusSignal.refreshAll(
        voltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        temperatureSignal,
        positionSignal,
        velocitySignal);
    log.appliedVolts = voltageSignal.getValueAsDouble();
    log.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
    log.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    log.temperatureCelsius = temperatureSignal.getValueAsDouble();
    log.position = positionSignal.getValueAsDouble();
    log.velocity = velocitySignal.getValueAsDouble();

    log.errorCode = voltageSignal.getStatus();

    log.licenseFault = licenseFaultSignal.getValue();

    return log;
  }
}
