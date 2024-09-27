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
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.utils.Tracer;
import java.util.Arrays;
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
public class PhoenixOdometryThread extends Thread implements OdometryThreadIO {
  public enum SignalType {
    DRIVE,
    STEER,
    GYRO;
    // In theory we could measure other types of info in this thread
    // But we don't!
  }

  /** modID should be GYRO_MODULE_ID for the gyro signal */
  public record SignalID(SignalType type, int modID) {}

  public record Registration(
      ParentDevice device,
      Optional<ModuleConstants> moduleConstants,
      SignalType type,
      Set<StatusSignal<Double>> signals) {}

  /** modID should be GYRO_MODULE_ID for the gyro signal */
  public record RegisteredSignal(StatusSignal<Double> signal, int modID, SignalType type) {}

  public record Samples(double timestamp, Map<SignalID, Double> values) {}

  private final ReadWriteLock journalLock = new ReentrantReadWriteLock(true);

  private final Set<RegisteredSignal> registeredSignals = Sets.newHashSet();
  private StatusSignal<Double>[] signalArr = new StatusSignal[0];
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

        registeredSignals.addAll(
            registration.signals.stream()
                .map(
                    s ->
                        new RegisteredSignal(
                            s,
                            registration.moduleConstants().isPresent()
                                ? registration.moduleConstants().get().id()
                                : GYRO_MODULE_ID,
                            registration.type()))
                .toList());
        registration.signals.stream()
            .forEach(
                (s) -> {
                  signalArr = Arrays.copyOf(signalArr, signalArr.length + 1);
                  signalArr[signalArr.length - 1] = s;
                });
      }
    } finally {
      writeLock.unlock();
    }
  }

  public List<Samples> samplesSince(double timestamp) {
    Tracer.startTrace("samples since");
    var readLock = journalLock.readLock();
    try {
      readLock.lock();

      return Tracer.traceFunc(
          "stream timestamps",
          () ->
              journal.stream()
                  .filter(s -> s.timestamp > timestamp)
                  .collect(Collectors.toUnmodifiableList()));
    } finally {
      readLock.unlock();
      Tracer.endTrace();
    }
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      var writeLock = journalLock.writeLock();
      Tracer.startTrace("Odometry Thread");
      Tracer.traceFunc(
          "wait for all",
          () -> BaseStatusSignal.waitForAll(2.0 / Module.ODOMETRY_FREQUENCY_HZ, signalArr));
      try {
        writeLock.lock();
        var filteredSignals =
            registeredSignals.stream()
                .filter(s -> s.signal().getStatus().equals(StatusCode.OK))
                .collect(Collectors.toSet());
        journal.add(
            new Samples(
                timestampFor(filteredSignals),
                filteredSignals.stream()
                    .collect(
                        Collectors.toUnmodifiableMap(
                            s -> new SignalID(s.type(), s.modID()),
                            s -> s.signal().getValueAsDouble()))));
      } finally {
        writeLock.unlock();
      }
      Tracer.endTrace();
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

  @Override
  public void updateInputs(OdometryThreadIOInputs inputs, double lastTimestamp) {
    inputs.sampledStates = samplesSince(lastTimestamp);
  }

  @Override
  public void start() {
    super.start();
  }
}
