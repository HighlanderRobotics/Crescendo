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
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.google.common.collect.EvictingQueue;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
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
  public record Registration(ParentDevice device, Set<StatusSignal<Double>> signals) {}

  public record Samples(double timestamp, Map<StatusSignal<Double>, Double> values) {}

  private final ReadWriteLock journalLock = new ReentrantReadWriteLock();

  private final Set<StatusSignal<Double>> signals = Sets.newHashSet();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();
  private final Queue<Samples> journal;

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

        signals.addAll(registration.signals);
      }
    } finally {
      writeLock.unlock();
    }
  }

  public List<Samples> samplesSince(double timestamp, Set<StatusSignal<Double>> signals) {
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
      var status =
          BaseStatusSignal.waitForAll(
              2.0 / Module.ODOMETRY_FREQUENCY_HZ, signals.toArray(new BaseStatusSignal[0]));
      try {
        writeLock.lock();
        if (status.isOK()) {
          journal.add(
              new Samples(timestampFor(signals), Maps.asMap(signals, s -> s.getValueAsDouble())));
        } else {
          // System.out.println("Odo thread error: " + status.toString());
        }
      } finally {
        writeLock.unlock();
      }
    }
  }

  private double timestampFor(Set<StatusSignal<Double>> signals) {
    final double totalTimestamp =
        signals.stream().mapToDouble(s -> s.getTimestamp().getTime()).sum();

    // Account for mean latency for a "good enough" timestamp
    if (!signals.isEmpty()) {
      return totalTimestamp / signals.size();
    } else {
      return Logger.getRealTimestamp() / 1e6;
    }
  }
}
