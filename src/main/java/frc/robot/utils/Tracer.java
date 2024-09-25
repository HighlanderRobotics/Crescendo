package frc.robot.utils;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A Utility class for tracing code execution time. Will put info to NetworkTables under the
 * "Tracer" table.
 *
 * <pre><code>
 *
 * @Override
 * public void loopFunc() {
 *    Tracer.traceFunc("LoopFunc", super::loopFunc);
 * }
 *
 * <p>
 *
 * @Override
 * public void robotPeriodic() {
 *     Tracer.startTrace("RobotPeriodic");
 *     Tracer.traceFunc("CommandScheduler", scheduler::run);
 *     Tracer.traceFunc("Monologue", Monologue::updateAll);
 *     Tracer.endTrace();
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
    private final NetworkTable rootTable;

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
    // the publishers for each trace, the key is the trace name, modified every endCycle
    private final HashMap<String, DoublePublisher> publisherHeap = new HashMap<>();

    // the garbage collector beans
    private final ArrayList<GarbageCollectorMXBean> gcs =
        new ArrayList<>(ManagementFactory.getGarbageCollectorMXBeans());
    private final DoublePublisher gcTimeEntry;
    private double gcTimeThisCycle = 0.0;

    private TracerState(String threadName) {
      this.rootTable = NetworkTableInstance.getDefault().getTable("Tracer").getSubTable(threadName);
      this.gcTimeEntry =
          rootTable.getDoubleTopic("GCTime").publishEx("double", "{ \"cached\": false}");
    }

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
      // update times for all already existing publishers
      for (var publisher : publisherHeap.entrySet()) {
        // if the entry isn't found, time will null-cast to 0.0
        Double time = traceTimes.remove(publisher.getKey());
        if (time == null) time = 0.0;
        Logger.recordOutput("Tracer/" + publisher.getKey(), time);
      }
      // create publishers for all new entries
      for (var traceTime : traceTimes.entrySet()) {
        DoublePublisher publisher =
            rootTable
                .getDoubleTopic(traceTime.getKey())
                .publishEx("double", "{ \"cached\": false}");
        publisher.set(traceTime.getValue());
        Logger.recordOutput("Tracer/" + traceTime.getKey(), traceTime.getValue());
        publisherHeap.put(traceTime.getKey(), publisher);
      }

      // log gc time
      if (gcs.size() > 0) gcTimeEntry.set(gcTimeThisCycle);
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
    state.gcTimeEntry.close();
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
   * Starts a trace, should be called at the beginning of a function thats not being called by user
   * code. Should be paired with {@link Tracer#endTrace()} at the end of the function.
   *
   * <p>Best used in periodic functions in Subsystems and Robot.java.
   *
   * @param name the name of the trace, should be unique to the function.
   */
  public static void startTrace(String name) {
    startTrace(name, threadLocalState.get());
  }

  /**
   * Ends a trace, should only be called at the end of a function thats not being called by user
   * code. If a {@link Tracer#startTrace(String)} is not paired with a {@link Tracer#endTrace()}
   * there could be a crash.
   */
  public static void endTrace() {
    endTrace(threadLocalState.get());
  }

  /**
   * Traces a function, should be used in place of {@link Tracer#startTrace(String)} and {@link
   * Tracer#endTrace()} for functions called by user code like {@code CommandScheduler.run()} and
   * other expensive functions.
   *
   * @param name the name of the trace, should be unique to the function.
   * @param runnable the function to trace.
   * @apiNote If you want to return a value then use {@link Tracer#traceFunc(String, Supplier)}.
   */
  public static void traceFunc(String name, Runnable runnable) {
    final TracerState state = threadLocalState.get();
    startTrace(name, state);
    runnable.run();
    endTrace(state);
  }

  /**
   * Traces a function, should be used in place of {@link Tracer#startTrace(String)} and {@link
   * Tracer#endTrace()} for functions called by user code like {@code CommandScheduler.run()} and
   * other expensive functions.
   *
   * @param name the name of the trace, should be unique to the function.
   * @param supplier the function to trace.
   */
  public static <T> T traceFunc(String name, Supplier<T> supplier) {
    final TracerState state = threadLocalState.get();
    startTrace(name, state);
    T ret = supplier.get();
    endTrace(state);
    return ret;
  }
}
