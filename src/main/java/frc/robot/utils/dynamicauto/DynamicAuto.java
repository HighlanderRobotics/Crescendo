package frc.robot.utils.dynamicauto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class DynamicAuto {

  public static final Note[] notes = {
    new Note(new Pose2d(2.204, 7.0, Rotation2d.fromRadians(-Math.PI)), true, 0), // w1
    new Note(new Pose2d(2.204, 5.5, Rotation2d.fromRadians(-Math.PI)), false, 1), // w2
    new Note(new Pose2d(2.204, 4.1, Rotation2d.fromRadians(-Math.PI)), false, 2), // w3
    new Note(new Pose2d(7.538, 7.3, Rotation2d.fromRadians(-Math.PI)), true, 0), // c1
    new Note(new Pose2d(7.538, 5.7, Rotation2d.fromRadians(-Math.PI)), false, 0), // c2
    new Note(new Pose2d(7.538, 4.1, Rotation2d.fromRadians(-Math.PI)), false, 1), // c3
    new Note(new Pose2d(7.538, 2.5, Rotation2d.fromRadians(-Math.PI)), false, 2), // c4
    new Note(new Pose2d(7.538, 0.7, Rotation2d.fromRadians(-Math.PI)), true, 3), // c5
  };

  public static final Pose2d[] shootingLocations = {
    new Pose2d(5.639, 6.463, Rotation2d.fromRadians(0.1)), // Left
    new Pose2d(4.216, 5.216, Rotation2d.fromRadians(0)), // Middle
    new Pose2d(6.048, 1.746, Rotation2d.fromRadians(-0.179)) // Right
  };

  public static Pose2d closestShootingLocation(Pose2d curPose) {
    Pose2d closestPosition = new Pose2d();
    double shortestDistance = 9999999;
    for (Pose2d location : shootingLocations) {
      if (shortestDistance > curPose.minus(location).getTranslation().getNorm()) {
        closestPosition = location;
        shortestDistance = curPose.minus(location).getTranslation().getNorm();
      }
    }
    return closestPosition;
  }

  public static PathPlannerPath makePath() {
    List<Translation2d> fgishf = PathPlannerPath.bezierFromPoses();
    return new PathPlannerPath(fgishf, null, null, null, null, null, false, null);
  }

  public static Note getClosestNote(Pose2d curPose) {
    Note closestNote = new Note();
    double shortestDistance = 99999999;
    for (Note note : notes) {
      System.out.println(note.getPriority());
      System.out.println(closestNote.getPriority());
      if (note.getBlacklist()) {
        System.out.println("iojfeioeoi");
        continue;

      } else if (note.getPriority() > closestNote.getPriority()) {
        shortestDistance = curPose.minus(note.getPose()).getTranslation().getNorm();
        closestNote = note;
        if (shortestDistance > curPose.minus(note.getPose()).getTranslation().getNorm()) {
          shortestDistance = curPose.minus(note.getPose()).getTranslation().getNorm();
          closestNote = note;
          System.out.println("ghoghsghuhguhuihi");
        }
      } else if (note.getPriority() == closestNote.getPriority()) {
        if (shortestDistance > curPose.minus(note.getPose()).getTranslation().getNorm()) {
          shortestDistance = curPose.minus(note.getPose()).getTranslation().getNorm();
          closestNote = note;
          System.out.println("ghoghsghuhguhuihi");
        }
      }
    }

    return closestNote;
  }
}
