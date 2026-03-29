package org.team5924.frc2026.util;

import org.team5924.frc2026.Constants;

public class ElevatorUtil {

  private ElevatorUtil() {}

  public static double metersToRotations(double height) { // Double check the math here
    return height
      * Constants.HopperElevator.MOTOR_TO_MECHANISM
      / (2 * Math.PI * Constants.HopperElevator.PULLEY_RADIUS_METERS);
  }
  // conversion from motor Rotations to Meters
  public static double rotationsToMeters(double rotations) {
    return (rotations
      * 2
      * Math.PI
      * Constants.HopperElevator.PULLEY_RADIUS_METERS
      / Constants.HopperElevator.MOTOR_TO_MECHANISM);
  }
}
