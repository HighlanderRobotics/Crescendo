package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Thanks to oh-yes-0-fps and <a
 * href="https://github.com/McQuaidRobotics/2024_Crescendo/blob/main/src/main/java/com/igknighters/util/Tracer.java">FRC
 * 3173!</a>
 *
 * <p>A newer version of this class is being merged into WPILib for 2025, and should supersede this
 * file once merged. The WPILib version may not support logging to akit the way we want, revisit
 * then (might need to manually connect nt publishing to log)
 *
 * <p>A Utility class for tracing code execution time. Will put info to Advantagekit under "Tracer".
 *
 * <pre><code>
 *
 * @Override
 * public void loopFunc() {
 *    Tracer.trace("LoopFunc", super::loopFunc);
 * }
 *
 * <p>
 *
 * @Override
 * public void robotPeriodic() {
 *     Tracer.trace("CommandScheduler", scheduler::run);
 * }
 *
 * </code></pre>
 */
public class Tracer {
  private static final class TraceStartData {
    private double startTime = 0.0;
    private double startGCTotalTime = 0.0;

    private void set(double startTime, double startGCTotalTime) {
      this.startTime = startTime;
      this.startGCTotalTime = startGCTotalTime;
    }
  }

  /**
   * All of the tracers persistent state in a single object to be stored in a {@link ThreadLocal}.
   */
  private static final class TracerState {

    // the stack of traces, every startTrace will add to this stack
    // and every endTrace will remove from this stack
    private final ArrayList<String> traceStack = new ArrayList<>();
    // ideally we only need `traceStack` but in the interest of memory optimization
    // and string concatenation speed we store the history of the stack to reuse the stack names
    private final ArrayList<String> traceStackHistory = new ArrayList<>();
    // the time of each trace, the key is the trace name, modified every endTrace
    private final HashMap<String, Double> traceTimes = new HashMap<>();
    // the start time of each trace and the gc time at the start of the trace,
    // the key is the trace name, modified every startTrace and endTrace.
    private final HashMap<String, TraceStartData> traceStartTimes = new HashMap<>();
    private final ArrayList<String> entryArray = new ArrayList<>();

    // the garbage collector beans
    private final ArrayList<GarbageCollectorMXBean> gcs =
        new ArrayList<>(ManagementFactory.getGarbageCollectorMXBeans());
    private double gcTimeThisCycle = 0.0;

    private TracerState(String threadName) {}

    private String appendTraceStack(String trace) {
      traceStack.add(trace);
      StringBuilder sb = new StringBuilder();
      for (int i = 0; i < traceStack.size(); i++) {
        sb.append(traceStack.get(i));
        if (i < traceStack.size() - 1) {
          sb.append("/");
        }
      }
      String str = sb.toString();
      traceStackHistory.add(str);
      return str;
    }

    private String popTraceStack() {
      traceStack.remove(traceStack.size() - 1);
      return traceStackHistory.remove(traceStackHistory.size() - 1);
    }

    private double totalGCTime() {
      double gcTime = 0;
      for (GarbageCollectorMXBean gc : gcs) {
        gcTime += gc.getCollectionTime();
      }
      return gcTime;
    }

    private void endCycle() {
      // update times for all already existing entries
      for (var entry : entryArray) {
        // if the entry isn't found, time will null-cast to 0.0
        Double time = traceTimes.remove(entry);
        if (time == null) time = 0.0;
        Logger.recordOutput("Tracer/" + entry, time);
      }
      // log all new entries
      for (var traceTime : traceTimes.entrySet()) {
        Logger.recordOutput("Tracer/" + traceTime.getKey(), traceTime.getValue());
        entryArray.add(traceTime.getKey());
      }

      // log gc time
      if (gcs.size() > 0) Logger.recordOutput("Tracer/GCTime", gcTimeThisCycle);
      gcTimeThisCycle = 0.0;

      // clean up state
      traceTimes.clear();
      traceStackHistory.clear();
    }
  }

  private static final ThreadLocal<TracerState> threadLocalState =
      ThreadLocal.withInitial(
          () -> {
            return new TracerState(Thread.currentThread().getName());
          });

  public static void disableGcLoggingForCurrentThread() {
    TracerState state = threadLocalState.get();
    state.gcs.clear();
  }

  private static void startTrace(final String name, final TracerState state) {
    String stack = state.appendTraceStack(name);
    TraceStartData data = state.traceStartTimes.get(stack);
    if (data == null) {
      data = new TraceStartData();
      state.traceStartTimes.put(stack, data);
    }
    data.set(Logger.getRealTimestamp() / 1000.0, state.totalGCTime());
  }

  private static void endTrace(final TracerState state) {
    try {
      String stack = state.popTraceStack();
      var startData = state.traceStartTimes.get(stack);
      double gcTimeSinceStart = state.totalGCTime() - startData.startGCTotalTime;
      state.gcTimeThisCycle += gcTimeSinceStart;
      state.traceTimes.put(
          stack, Logger.getRealTimestamp() / 1000.0 - startData.startTime - gcTimeSinceStart);
      if (state.traceStack.size() == 0) {
        state.endCycle();
      }
    } catch (Exception e) {
      DriverStation.reportError(
          "[Tracer] An end trace was called with no opening trace " + e, true);
    }
  }

  /**
   * Traces a function.
   *
   * @param name the name of the trace, should be unique to the function.
   * @param runnable the function to trace.
   * @apiNote If you want to return a value then use {@link Tracer#trace(String, Supplier)}.
   */
  public static void trace(String name, Runnable runnable) {
    final TracerState state = threadLocalState.get();
    try {
      startTrace(name, state);
      runnable.run();
    } finally {
      endTrace(state);
    }
  }

  /**
   * Traces a function.
   *
   * @param name the name of the trace, should be unique to the function.
   * @param supplier the function to trace.
   */
  public static <T> T trace(String name, Supplier<T> supplier) {
    final TracerState state = threadLocalState.get();
    try {
      startTrace(name, state);
      T ret = supplier.get();
      return ret;
    } finally {
      endTrace(state);
    }
  }
}
