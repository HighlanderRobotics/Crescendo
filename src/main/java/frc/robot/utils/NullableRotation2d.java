// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

/** Wrapper over WPILib's Rotation2d to allow serialization of nulls.*/
public class NullableRotation2d {
    boolean isNull;
    Rotation2d value;

    public NullableRotation2d(Rotation2d value) {
      isNull = value == null ? true : false;
      this.value = value;
    }

    public Rotation2d get() {
      return isNull ? null : value;
    }

  public static final NullableRotation2dStruct struct = new NullableRotation2dStruct();
    
}