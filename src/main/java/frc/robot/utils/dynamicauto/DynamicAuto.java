package frc.robot.utils.dynamicauto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

  ///  public static final Pose2d

  //  public static PathPlannerPath makePath(){
  ///    List<Translation2d> fgishf = PathPlannerPath.bezierFromPoses();
  ///  }

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
