// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.Struct;

/** Wrapper over WPILib's Rotation2dStruct to allow serialization of nulls. */
public class NullableRotation2dStruct implements Struct<NullableRotation2d>{
    @Override
    public Class<NullableRotation2d> getTypeClass() {
      return NullableRotation2d.class;
    }

    @Override
    public String getTypeString() {
      return "struct:NullableRotation2d";
    }

    @Override
    public int getSize() {
      return kSizeDouble;
    }

    @Override
    public String getSchema() {
      return "double value";
    }

    @Override
    public NullableRotation2d unpack(ByteBuffer bb) {
      byte isNull = bb.get();
      if (isNull == 0) {
        return null;
      } else {
        double value = bb.getDouble();
        return new NullableRotation2d(new Rotation2d(value));
      }
    }

    @Override
    public void pack(ByteBuffer bb, NullableRotation2d value) {
      if (value.isNull) {
        bb.put((byte) 0); // i don't think i can put a boolean in a byte buffer
      } else {
        bb.put((byte) 1);
        bb.putDouble(value.get().getRadians());
      };
    }
  }