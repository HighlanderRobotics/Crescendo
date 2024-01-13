// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.Robot;

/** Test robot init. */
public class InitTests {
    @BeforeEach
    private void setup() {
        HAL.initialize(500, 0);
    }

    @Test
    private void initTest() {
        var robot = new Robot();
        robot.startCompetition();
        robot.close();
    }
  }
