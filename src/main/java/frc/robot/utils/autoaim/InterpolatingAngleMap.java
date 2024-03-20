// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autoaim;

import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class InterpolatingAngleMap {

    
  private final TreeMap<Double, Rotation2d> map = new TreeMap<>();

  public InterpolatingAngleMap() {}

  public void put(Double key, Rotation2d value) {
    map.put(key, value);
  }

  public Rotation2d get(Double key) {
    Rotation2d val = map.get(key);
    if (val == null) {
      Double ceilingKey = map.ceilingKey(key);
      Double floorKey = map.floorKey(key);

      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return map.get(floorKey);
      }
      if (floorKey == null) {
        return map.get(ceilingKey);
      }
      Rotation2d floor = map.get(floorKey);
      Rotation2d ceiling = map.get(ceilingKey);

      return interpolate(floor, ceiling, inverseInterpolate(ceilingKey, key, floorKey));
    } else {
      return val;
    }
  }

  public void clear() {
    map.clear();
  }

  public void remove(double key) {
    map.remove(key);
  }

  private Rotation2d interpolate(Rotation2d startValue, Rotation2d endValue, double t) {
    return Rotation2d.fromRotations(
        MathUtil.interpolate(
            startValue.getRotations(), endValue.getRotations(), t));
  }

  private double inverseInterpolate(Double up, Double q, Double down) {
    double upperToLower = up.doubleValue() - down.doubleValue();
    if (upperToLower <= 0) {
      return 0.0;
    }
    double queryToLower = q.doubleValue() - down.doubleValue();
    if (queryToLower <= 0) {
      return 0.0;
    }
    return queryToLower / upperToLower;
  }
}
