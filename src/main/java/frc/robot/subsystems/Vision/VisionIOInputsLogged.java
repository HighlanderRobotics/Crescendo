package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * AutoLog does not work with PhotonTrackedTargets as of yet which is very sad but until it does
 * here is a manual implementation
 */
public class VisionIOInputsLogged extends VisionIO.VisionIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Timestamp", timestamp);
    table.put("Latency", latency);
    for (int i = 0; i < targets.size(); i++) {
      VisionHelper.logPhotonTrackedTarget(targets.get(i), table, String.valueOf(i));
      numTags += 1;
    }
    table.put("NumTags", numTags);
    table.put("Pose", pose);
  }

  @Override
  public void fromLog(LogTable table) {
    timestamp = table.get("Timestamp", timestamp);
    latency = table.get("Latency", latency);
    for (int i = 0; i < table.get("number of tags", targets.size()); i++) {
      this.targets.add(VisionHelper.getLoggedPhotonTrackedTarget(table, String.valueOf(i)));
    }
    numTags = table.get("NumTags", numTags);
    pose = table.get("Pose", pose);
  }

  public VisionIOInputsLogged clone() {
    VisionIOInputsLogged copy = new VisionIOInputsLogged();
    copy.timestamp = this.timestamp;
    copy.latency = this.latency;
    copy.targets = this.targets;
    copy.numTags = this.numTags;
    copy.pose = this.pose;
    return copy;
  }
}
