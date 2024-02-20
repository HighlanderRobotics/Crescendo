package frc.robot.utils.dynamicauto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.google.common.base.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DynamicAuto {

  public static final Note[] notes = {
    new Note(new Pose2d(2.204, 7.0, Rotation2d.fromRadians(Math.PI)), false, 0, "W1"), // w1
    new Note(new Pose2d(2.204, 5.5, Rotation2d.fromRadians(Math.PI)), false, 1, "W2"), // w2
    new Note(new Pose2d(2.204, 4.1, Rotation2d.fromRadians(Math.PI)), false, 2, "W3"), // w3
    new Note(new Pose2d(7.538, 7.3, Rotation2d.fromRadians(Math.PI)), false, 0, "C1"), // c1
    new Note(new Pose2d(7.538, 5.7, Rotation2d.fromRadians(Math.PI)), false, 0, "C2"), // c2
    new Note(new Pose2d(7.538, 4.1, Rotation2d.fromRadians(Math.PI)), false, 1, "C3"), // c3
    new Note(new Pose2d(7.538, 2.5, Rotation2d.fromRadians(Math.PI)), false, 2, "C4"), // c4
    new Note(new Pose2d(7.538, 0.7, Rotation2d.fromRadians(Math.PI)), false, 3, "C5") // c5
  };

  public static final ShootingLocation[] shootingLocations = {
    new ShootingLocation(new Pose2d(5.639, 6.463, Rotation2d.fromRadians(-3.04)), "S1"), // Left
    new ShootingLocation(new Pose2d(4.216, 5.216, Rotation2d.fromRadians(-Math.PI)), "S2"),
    new ShootingLocation(new Pose2d(4.263, 3.000, Rotation2d.fromRadians(-0.566)), "S3"), // Middle
    new ShootingLocation(new Pose2d(5.176, 1.620, Rotation2d.fromRadians(2.961)), "S4") // Right
  };

  public static ShootingLocation closestShootingLocation(Supplier<Pose2d> curPose) {

    ShootingLocation closestLocation = new ShootingLocation();
    double shortestDistance = 9999999;
    for (ShootingLocation location : shootingLocations) {
      if (shortestDistance > curPose.get().minus(location.getPose()).getTranslation().getNorm()) {
        closestLocation = location;
        shortestDistance = curPose.get().minus(location.getPose()).getTranslation().getNorm();
      }
    }
    return closestLocation;
  }

  public static ChoreoTrajectory makeNoteToNote(Supplier<Pose2d> startingPose) {
    Note closestNote = getClosestNote(startingPose.get());
    closestNote.blacklist();
    Note nextClosest = getClosestNote(startingPose.get());
    closestNote.whitelist();
    
    return Choreo.getTrajectory(closestNote.getName() + " To " + nextClosest.getName());
  }

  public static Note getClosestNote(Pose2d curPose) {
    Note closestNote = new Note();
    double shortestDistance = 99999999;
    for (Note note : notes) {
      if (note.getBlacklist()) {

        continue;
      } else if (note.getPriority() > closestNote.getPriority()) {
        shortestDistance = curPose.minus(note.getPose()).getTranslation().getNorm();
        closestNote = note;
        if (shortestDistance > curPose.minus(note.getPose()).getTranslation().getNorm()) {
          shortestDistance = curPose.minus(note.getPose()).getTranslation().getNorm();
          closestNote = note;
        }
      } else if (note.getPriority() == closestNote.getPriority()) {
        if (shortestDistance > curPose.minus(note.getPose()).getTranslation().getNorm()) {
          shortestDistance = curPose.minus(note.getPose()).getTranslation().getNorm();
          closestNote = note;
        }
      }
    }

    return closestNote;
  }
}
