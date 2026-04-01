/*
 * FieldAwareness.java
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

package org.team5924.frc2026.subsystems.awareness;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.RobotState;

public class FieldAwareness {
  private static FieldAwareness instance;

  public static FieldAwareness getInstance() {
    if (instance == null) instance = new FieldAwareness();
    return instance;
  }

  public Translation2d[] getOffsetsToTrenches() {
    Translation2d[] offsets = new Translation2d[4];
    Translation2d robotPosition = RobotState.getInstance().getOdometryPose().getTranslation();

    for (int i = 0; i < 4; i++) {
      offsets[i] = Constants.Field.TRENCH_TRANSLATIONS[i].minus(robotPosition);
    }

    return offsets;
  }

  private void tdfyguijoi() { // TODO: name later
    ChassisSpeeds speeds = RobotState.getInstance().getRobotChassisSpeeds();
    Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    for (int i = 0; i < 4; i++) {}
  }
}
