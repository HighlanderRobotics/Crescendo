package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Queues;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Samples;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalID;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalType;
import java.util.Collections;
import java.util.Map;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class PhoenixOdometryThreadTest {
  private TalonFX talon;

  @BeforeEach
  void setup() {
    talon = new TalonFX(0);
  }

  @Test
  void statusSignalsShouldBeEqualIfRequestedTwice() {
    // This is to ensure that phoenix's StatusSignals are actually usable as hash keys
    Assertions.assertEquals(talon.getAcceleration(), talon.getAcceleration());
    Assertions.assertNotEquals(talon.getAcceleration(), talon.getPosition());

    Assertions.assertEquals(talon.getAcceleration().hashCode(), talon.getAcceleration().hashCode());
  }

  @Test
  void statusSignalsFromDifferentDevicesShouldNotBeEqual() {
    TalonFX otherTalon = new TalonFX(1);
    Assertions.assertNotEquals(talon.getAcceleration(), otherTalon.getAcceleration());

    Assertions.assertNotEquals(
        talon.getAcceleration().hashCode(), otherTalon.getAcceleration().hashCode());
  }

  @Test
  void samplesSinceReturnsNothingWhenNoSamples() {
    var thread = PhoenixOdometryThread.createWithJournal(Queues.newArrayDeque());
    var samples = thread.samplesSince(0.0);
    Assertions.assertEquals(Collections.emptyList(), samples);
  }

  @Test
  void samplesSinceReturnsAfterTimestamp() {
    Map<SignalID, Double> sampleValue = ImmutableMap.of(new SignalID(SignalType.GYRO, OdometryThreadIO.GYRO_MODULE_ID), 42.0);
    var sample = new Samples(10, sampleValue);
    var thread =
        PhoenixOdometryThread.createWithJournal(Queues.newArrayDeque(ImmutableList.of(sample)));
    var samples = thread.samplesSince(0);
    Assertions.assertEquals(ImmutableList.of(sample), samples);
  }

  @Test
  void samplesSinceIgnoresBeforeTimestamp() {
    Map<SignalID, Double> sampleValue = ImmutableMap.of(new SignalID(SignalType.GYRO, OdometryThreadIO.GYRO_MODULE_ID), 42.0);
    var sample = new Samples(10, sampleValue);

    var thread =
        PhoenixOdometryThread.createWithJournal(Queues.newArrayDeque(ImmutableList.of(sample)));
    var samples = thread.samplesSince(15);
    Assertions.assertEquals(Collections.emptyList(), samples);
  }

  @Test
  void samplesSinceIgnoresWhenTimestampEqual() {
    Map<SignalID, Double> sampleValue = ImmutableMap.of(new SignalID(SignalType.GYRO, OdometryThreadIO.GYRO_MODULE_ID), 42.0);
    var sample = new Samples(10, sampleValue);
    var thread =
        PhoenixOdometryThread.createWithJournal(Queues.newArrayDeque(ImmutableList.of(sample)));
    var samples = thread.samplesSince(10);
    Assertions.assertEquals(Collections.emptyList(), samples);
  }
}
