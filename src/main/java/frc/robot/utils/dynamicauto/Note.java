// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.dynamicauto;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class Note {

  private boolean blacklisted;
  private int priority;
  private final Pose2d pose;

  public Note(Pose2d pose, boolean blacklisted, int priority) {
    this.pose = pose;
    this.blacklisted = blacklisted;
    this.priority = priority;
  }

  public Note() {
    this.pose = new Pose2d();
    this.blacklisted = true;
    this.priority = -1;
  }

  public void setBlacklist(boolean blacklist) {
    this.blacklisted = blacklist;
  }

  public void setPriority(Integer priority) {
    this.priority = priority;
  }

  public boolean getBlacklist() {
    return blacklisted;
  }

  public int getPriority() {
    return priority;
  }

  public Pose2d getPose() {
    return pose;
  }
}
