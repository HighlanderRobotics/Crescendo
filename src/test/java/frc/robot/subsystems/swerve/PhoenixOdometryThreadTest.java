package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Queues;
import com.google.common.collect.Sets;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Samples;
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
  void samplesSinceReturnsNothingWhenNoSamples() {
    var thread = PhoenixOdometryThread.createWithJournal(Queues.newArrayDeque());
    var samples = thread.samplesSince(0, Sets.newHashSet(talon.getAcceleration()));
    Assertions.assertEquals(Collections.emptyList(), samples);
  }

  @Test
  void samplesSinceReturnsAfterTimestamp() {
    Map<BaseStatusSignal, Double> sampleValue = ImmutableMap.of(talon.getAcceleration(), 42.0);
    var sample = new Samples(10, sampleValue);
    var thread =
        PhoenixOdometryThread.createWithJournal(Queues.newArrayDeque(ImmutableList.of(sample)));
    var samples = thread.samplesSince(0, Sets.newHashSet(talon.getAcceleration()));
    Assertions.assertEquals(ImmutableList.of(sample), samples);
  }

  @Test
  void samplesSinceIgnoresBeforeTimestamp() {
    Map<BaseStatusSignal, Double> sampleValue = ImmutableMap.of(talon.getAcceleration(), 42.0);
    var sample = new Samples(10, sampleValue);
    var thread =
        PhoenixOdometryThread.createWithJournal(Queues.newArrayDeque(ImmutableList.of(sample)));
    var samples = thread.samplesSince(15, Sets.newHashSet(talon.getAcceleration()));
    Assertions.assertEquals(Collections.emptyList(), samples);
  }

  @Test
  void samplesSinceIgnoresWhenTimestampEqual() {
    Map<BaseStatusSignal, Double> sampleValue = ImmutableMap.of(talon.getAcceleration(), 42.0);
    var sample = new Samples(10, sampleValue);
    var thread =
        PhoenixOdometryThread.createWithJournal(Queues.newArrayDeque(ImmutableList.of(sample)));
    var samples = thread.samplesSince(10, Sets.newHashSet(talon.getAcceleration()));
    Assertions.assertEquals(Collections.emptyList(), samples);
  }
}
