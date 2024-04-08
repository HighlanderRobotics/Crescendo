// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;

/** Silly */
public class NullableDoubleStruct implements Struct<NullableDouble> {
    @Override
  public Class<NullableDouble> getTypeClass() {
    return NullableDouble.class;
  }

  @Override
  public String getTypeString() {
    return "struct:NullableDouble";
  }

  @Override
  public int getSize() {
    return kSizeDouble + kSizeBool;
  }

  @Override
  public String getSchema() {
    return "boolean isNull; double value;";
  }

  @Override
  public NullableDouble unpack(ByteBuffer bb) {
    byte isNull = bb.get();
    double value = bb.getDouble();
    if (isNull == 0) {
      return new NullableDouble(null);
    } else {
      return new NullableDouble(value);
    }
  }

  @Override
  public void pack(ByteBuffer bb, NullableDouble value) {
    if (value.isNull) {
      bb.put((byte) 0);
      bb.putDouble(0);
    } else {
      bb.put((byte) 1);
      bb.putDouble(value.get());
    }
  }
}
