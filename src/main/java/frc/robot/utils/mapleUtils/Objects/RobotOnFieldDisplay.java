package frc.robot.utils.mapleUtils.Objects;

import frc.robot.utils.mapleUtils.CompetitionFieldVisualizer;

/**
 * displays a robot on field note that the main robot also inherits this class, but it will not be
 * displayed in "Robots" Pose3d array it will be displayed as "Robot" and with a single Pose2d
 */
public interface RobotOnFieldDisplay extends CompetitionFieldVisualizer.Object2dOnFieldDisplay {
  @Override
  default String getTypeName() {
    return "Robots";
  }
}
