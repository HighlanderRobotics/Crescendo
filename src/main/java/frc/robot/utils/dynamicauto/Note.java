// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.dynamicauto;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class Note {

  private boolean blacklisted;
  private int priority;
  private Pose2d pose;
  private final String name;
  private boolean isThere;

  public Note(Pose2d pose, boolean blacklisted, int priority, String name, boolean isThere) {
    this.pose = pose;
    this.blacklisted = blacklisted;
    this.priority = priority;
    this.name = name;
    this.isThere = isThere;
  }

  public Note(Pose2d pose, boolean blacklisted, int priority, String name) {
    this.pose = pose;
    this.blacklisted = blacklisted;
    this.priority = priority;
    this.name = name;
    this.isThere = false;
  }

  public Note(String name) {
    this.pose = new Pose2d();
    this.blacklisted = true;
    this.priority = -1;
    this.name = name;
    this.isThere = false;
  }

  public Note() {
    this.pose = new Pose2d();
    this.blacklisted = true;
    this.priority = -1;
    this.name = "Uninitialized";
    this.isThere = false;
  }

  public void blacklist() {
    System.out.println("Blacklisted " + name);
    this.blacklisted = true;
  }

  public void whitelist() {
    System.out.println("Whitelisted " + name);
    this.blacklisted = false;
  }

  public void setPriority(Integer priority) {
    this.priority = priority;
  }

  public void setPose(Pose2d pose){
    this.pose = pose;
  }

  public boolean getBlacklist() {
    return blacklisted;
  }

  public boolean getExistence() {
    return isThere;
  }

  public int getPriority() {
    return priority;
  }

  public Pose2d getPose() {
    return pose;
  }

  public String getName() {
    return name;
  }
}
