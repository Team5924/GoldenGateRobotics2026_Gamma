/*
 * ElevatorUtil.java
 */

/* 
 * Copyright (C) 2025-2026 Team 5924 - Golden Gate Robotics and/or its affiliates.
 *
 * This file, and the associated project, are offered under the GNU General
 * Public License v3.0. A copy of this license can be found in LICENSE.md
 * at the root of this project.
 *
 * If this file has been separated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

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
