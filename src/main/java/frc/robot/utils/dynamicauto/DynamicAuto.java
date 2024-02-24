package frc.robot.utils.dynamicauto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.google.common.base.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;

public class DynamicAuto {

  public static final Note[] notes = {
    new Note(new Pose2d(2.204, 7.0, Rotation2d.fromRadians(Math.PI)), false, 0, "W1", true), // w1
    new Note(new Pose2d(2.204, 5.5, Rotation2d.fromRadians(Math.PI)), false, 1, "W2", true), // w2
    new Note(new Pose2d(2.204, 4.1, Rotation2d.fromRadians(Math.PI)), false, 2, "W3", true), // w3
    new Note(new Pose2d(7.538, 7.3, Rotation2d.fromRadians(Math.PI)), false, 3, "C1", true), // c1
    new Note(new Pose2d(7.538, 5.7, Rotation2d.fromRadians(Math.PI)), false, 0, "C2", true), // c2
    new Note(new Pose2d(7.538, 4.1, Rotation2d.fromRadians(Math.PI)), false, 1, "C3", true), // c3
    new Note(new Pose2d(7.538, 2.5, Rotation2d.fromRadians(Math.PI)), false, 2, "C4", true), // c4
    new Note(new Pose2d(7.538, 0.7, Rotation2d.fromRadians(Math.PI)), false, 0, "C5", true) // c5
  };

  public static final ShootingLocation[] shootingLocations = {
    new ShootingLocation(new Pose2d(5.639, 6.463, Rotation2d.fromRadians(-3.04)), "S1"), // Left
    new ShootingLocation(new Pose2d(4.216, 5.216, Rotation2d.fromRadians(-Math.PI)), "S2"),
    new ShootingLocation(new Pose2d(4.263, 3.000, Rotation2d.fromRadians(-0.566)), "S3"), // Middle
    new ShootingLocation(new Pose2d(5.176, 1.620, Rotation2d.fromRadians(2.961)), "S4") // Right
  };

  public static final ShootingLocation[] startingLocations = {
    new ShootingLocation(
        new Pose2d(0.71, 6.72, Rotation2d.fromRadians(1.04)), "Amp Side"), // Speaker (Amp Side)
    new ShootingLocation(
        new Pose2d(1.35, 5.56, Rotation2d.fromRadians(0)), "Center"), // Speaker (Center)
    new ShootingLocation(
        new Pose2d(0.71, 4.36, Rotation2d.fromRadians(-1.04)), "Stage Side") // Speaker (Stage Side)
  };

  public static int whitelistCount = notes.length;

  public static void updateWhitelistCount() {
    whitelistCount = 0;
    for (Note note : notes) {
      whitelistCount += note.getBlacklist() ? 1 : 0;
    }
  }

  public static void updateNote(Note note, BooleanSupplier blacklist, IntSupplier priority) {
    if (blacklist.getAsBoolean()) {
      note.blacklist();
    } else {
      note.whitelist();
    }
    note.setPriority(priority.getAsInt());
  }

  public static Command DynamicToNote(Supplier<Pose2d> curPose) {
    return AutoBuilder.pathfindToPose(
        getClosestNote(curPose).getPose(),
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
  }

  public static Command DynamicToShoot(Supplier<Pose2d> curPose) {
    return AutoBuilder.pathfindToPose(
        closestShootingLocation(curPose, shootingLocations).getPose(),
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
  }

  public static ShootingLocation closestShootingLocation(
      Supplier<Pose2d> curPose, ShootingLocation[] shootingLocations) {

    ShootingLocation closestLocation = new ShootingLocation("shooting shooting");
    double shortestDistance = 9999999;
    for (ShootingLocation location : shootingLocations) {
      if (shortestDistance > curPose.get().minus(location.getPose()).getTranslation().getNorm()) {
        closestLocation = location;
        shortestDistance = curPose.get().minus(location.getPose()).getTranslation().getNorm();
      }
    }
    return closestLocation;
  }

  public static void flipNote(Note note) {
    Pose2d pose = note.getPose();
    note.setPose(
        new Pose2d(
            16.5410515 - pose.getX(),
            pose.getY(),
            Rotation2d.fromRadians(Math.PI - pose.getRotation().getRadians())));
  }

  public static ChoreoTrajectory makeStartToNote(Supplier<Pose2d> startingPose) {
    ShootingLocation startingLocation = closestShootingLocation(startingPose, startingLocations);
    Note closestNote = getClosestNote(startingPose);
    if ("Uninitialized" == closestNote.getName()) {
      closestNote = getAbsoluteClosestNote(startingPose);
      System.out.println("No more avaliable notes!");
    }
    return Choreo.getTrajectory(startingLocation.getName() + " To " + closestNote.getName());
  }

  public static ChoreoTrajectory makeNoteToShooting(Supplier<Pose2d> startingPose) {
    Note closestNote = getAbsoluteClosestNote(startingPose);
    ShootingLocation startingLocation = closestShootingLocation(startingPose, shootingLocations);
    if ("Uninitialized" == closestNote.getName()) {
      closestNote = getAbsoluteClosestNote(startingPose);
      System.out.println("No more avaliable notes!");
    }
    return Choreo.getTrajectory(closestNote.getName() + " To " + startingLocation.getName());
  }

  public static ChoreoTrajectory makeShootingToNote(Supplier<Pose2d> startingPose) {
    Note closestNote = getClosestNote(startingPose);
    ShootingLocation startingLocation = closestShootingLocation(startingPose, shootingLocations);
    if ("Uninitialized" == closestNote.getName()) {
      closestNote = getAbsoluteClosestNote(startingPose);
      System.out.println("No more avaliable notes!");
    }
    return Choreo.getTrajectory(startingLocation.getName() + " To " + closestNote.getName());
  }

  public static ChoreoTrajectory makeNoteToNote(Supplier<Pose2d> startingPose) {
    Note closestNote = getAbsoluteClosestNote(startingPose);
    Note nextClosest = new Note();
    if (!closestNote.getBlacklist()) {
      closestNote.blacklist();
      nextClosest = getClosestNote(startingPose);
      closestNote.whitelist();
    } else {
      nextClosest = getClosestNote(startingPose);
    }

    Logger.recordOutput(
        "DynamicAuto/Path Name", closestNote.getName() + " To " + nextClosest.getName());
    return Choreo.getTrajectory(closestNote.getName() + " To " + nextClosest.getName());
  }

  public static Note getAbsoluteClosestNote(Supplier<Pose2d> curPose) {
    Note closestNote = new Note();
    double shortestDistance = 99999999;
    for (Note note : notes) {
      if (shortestDistance > curPose.get().minus(note.getPose()).getTranslation().getNorm()) {
        shortestDistance = curPose.get().minus(note.getPose()).getTranslation().getNorm();
        closestNote = note;
      }
    }

    return closestNote;
  }

  public static Note getClosestNote(Supplier<Pose2d> curPose) {
    Note closestNote = new Note();
    double shortestDistance = Double.POSITIVE_INFINITY;
    for (Note note : notes) {
      if (note.getBlacklist()) {

        continue;
      } else if (note.getPriority() > closestNote.getPriority()) {
        shortestDistance = curPose.get().minus(note.getPose()).getTranslation().getNorm();
        closestNote = note;
        if (shortestDistance > curPose.get().minus(note.getPose()).getTranslation().getNorm()) {
          shortestDistance = curPose.get().minus(note.getPose()).getTranslation().getNorm();
          closestNote = note;
        }
      } else if (note.getPriority() == closestNote.getPriority()) {
        if (shortestDistance > curPose.get().minus(note.getPose()).getTranslation().getNorm()) {
          shortestDistance = curPose.get().minus(note.getPose()).getTranslation().getNorm();
          closestNote = note;
        }
      }
    }

    return closestNote;
  }
}
