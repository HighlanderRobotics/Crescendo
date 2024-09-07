// Copyright 2021-2024 FRC 6328, FRC 8033, Kevin Clark
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.google.common.collect.EvictingQueue;
import com.google.common.collect.Sets;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.swerve.Module.ModuleConstants;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for high frequency sampling of Phoenix devices.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. CAN FD
 * compliant devices are required (TalonFX, CANCoder, Pigeon 2.0, CANdle) due to the use of the
 * "waitForAll" blocking method to enable more consistent sampling. This also allows Phoenix Pro
 * users to benefit from lower latency between devices using CANivore time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
  public enum SignalType {
    Drive,
    Steer,
    Gyro;
    // In theory we could measure other types of info in this thread
    // But we don't!
  }

  public record SignalID(SignalType type, int modID) {}

  public record Registration(ParentDevice device, Optional<ModuleConstants> moduleConstants, SignalType type, Set<StatusSignal<Double>> signals) {}

  public record RegisteredSignal(StatusSignal<Double> signal, Optional<ModuleConstants> moduleConstants, SignalType type) {}

  public record Samples(double timestamp, Map<SignalID, Double> values) {}

  public record SampledPositions(double timestamp, Map<Integer, SwerveModulePosition> positions) {}

  private final ReadWriteLock journalLock = new ReentrantReadWriteLock(true);

  private final Set<RegisteredSignal> signals = Sets.newHashSet();
  private final Queue<Samples> journal;

  // For gyros
  private static final ModuleConstants NEGATIVE_ONE = new ModuleConstants(-1, "", -1, -1, -1, Rotation2d.fromRotations(0));

  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  // Used for testing
  protected static PhoenixOdometryThread createWithJournal(final Queue<Samples> journal) {
    return new PhoenixOdometryThread(journal);
  }

  private PhoenixOdometryThread(final Queue<Samples> journal) {
    setName("PhoenixOdometryThread");
    setDaemon(true);

    this.journal = journal;
  }

  private PhoenixOdometryThread() {
    this(EvictingQueue.create(20));
  }

  public void registerSignals(Collection<Registration> registrations) {
    registerSignals(registrations.toArray(new Registration[] {}));
  }

  // Returns a handle which can be used to collect the last 20 signal results
  public void registerSignals(Registration... registrations) {
    var writeLock = journalLock.writeLock();

    try {
      writeLock.lock();

      for (var registration : registrations) {
        assert CANBus.isNetworkFD(registration.device.getNetwork()) : "Only CAN FDs supported";

        signals.addAll(registration.signals.stream().map(s -> new RegisteredSignal(s, registration.moduleConstants(), registration.type())).toList());
      }
    } finally {
      writeLock.unlock();
    }
  }

  public List<Samples> samplesSince(double timestamp, Set<RegisteredSignal> signals) {
    var readLock = journalLock.readLock();
    try {
      readLock.lock();

      return journal.stream()
          .filter(s -> s.timestamp > timestamp)
          .map(
              s -> {
                var filteredValues =
                    s.values.entrySet().stream()
                        .filter(e -> signals.contains(e.getKey()))
                        .collect(
                            Collectors.toUnmodifiableMap(Map.Entry::getKey, Map.Entry::getValue));
                return new Samples(s.timestamp, filteredValues);
              })
          .collect(Collectors.toUnmodifiableList());
    } finally {
      readLock.unlock();
    }
  }

  public List<Samples> samplesSince(double timestamp) {
    return samplesSince(timestamp, signals);
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      var writeLock = journalLock.writeLock();
      // NOTE (kevinclark): The toArray here in a tight loop is kind of ugly
      // but keeping up a symmetric array is too and it's probably negligible on latency.
      BaseStatusSignal.waitForAll(
          2.0 / Module.ODOMETRY_FREQUENCY_HZ, signals.toArray(new BaseStatusSignal[0]));
      try {
        writeLock.lock();
        var filteredSignals =
            signals.stream()
                .filter(s -> s.signal().getStatus().equals(StatusCode.OK))
                .collect(Collectors.toSet());
        journal.add(
            new Samples(
                timestampFor(filteredSignals),
                filteredSignals.stream()
                    .collect(Collectors.toUnmodifiableMap(s -> new SignalID(s.type(), s.moduleConstants().orElse(NEGATIVE_ONE).id()), s -> s.signal().getValueAsDouble()))));
      } finally {
        writeLock.unlock();
      }
    }
  }

  private double timestampFor(Set<RegisteredSignal> signals) {
    double timestamp = Logger.getRealTimestamp() / 1e6;

    final double totalLatency =
        signals.stream().mapToDouble(s -> s.signal().getTimestamp().getLatency()).sum();

    // Account for mean latency for a "good enough" timestamp
    if (!signals.isEmpty()) {
      timestamp -= totalLatency / signals.size();
    }

    return timestamp;
  }
}
