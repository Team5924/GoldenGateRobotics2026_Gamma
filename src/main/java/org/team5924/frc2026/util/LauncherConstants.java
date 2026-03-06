// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.team5924.frc2026.util;

import org.team5924.frc2026.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class LauncherConstants {
  public static Transform3d robotToLauncherLeft =
      new Transform3d(Constants.TurretLeft.ROBOT_TO_TURRET, new Rotation3d(0.0, 0.0, Math.PI)); // TODO: update

  public static Transform3d robotToLauncherRight =
      new Transform3d(Constants.TurretRight.ROBOT_TO_TURRET, new Rotation3d(0.0, 0.0, Math.PI)); // TODO: update

  private LauncherConstants() {}
}