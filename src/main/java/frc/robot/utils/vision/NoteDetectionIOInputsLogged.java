package frc.robot.utils.vision;

import frc.robot.subsystems.vision.NoteDetectionIO;
import frc.robot.subsystems.vision.VisionHelper;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class NoteDetectionIOInputsLogged extends NoteDetectionIO.NoteDetectionIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Timestamp", timestamp);
    table.put("Latency", latency);
    for (int i = 0; i < targets.size(); i++) {
      VisionHelper.Logging.logPhotonTrackedTarget(targets.get(i), table, String.valueOf(i));
    }
    table.put("NumTargets", targets.size());
    VisionHelper.Logging.logVisionConstants(constants, table);
    table.put("Note Yaw", yaw);
    table.put("Note Pitch", pitch);
    table.put("Note Distance", distance);
  }

  @Override
  public void fromLog(LogTable table) {
    timestamp = table.get("Timestamp", timestamp);
    latency = table.get("Latency", latency);
    for (int i = 0; i < table.get("NumTargets", numTargets); i++) {
      this.targets.add(VisionHelper.Logging.getLoggedPhotonTrackedTarget(table, String.valueOf(i)));
    }
    constants = VisionHelper.Logging.getLoggedVisionConstants(table);
    yaw = table.get("Note Yaw", yaw);
    pitch = table.get("Note Pitch", pitch);
    distance = table.get("Note Distance", distance);
  }

  public NoteDetectionIOInputsLogged clone() {
    NoteDetectionIOInputsLogged copy = new NoteDetectionIOInputsLogged();
    copy.timestamp = this.timestamp;
    copy.latency = this.latency;
    copy.yaw = this.yaw;
    copy.pitch = this.pitch;
    copy.distance = this.distance;
    return copy;
  }
}
