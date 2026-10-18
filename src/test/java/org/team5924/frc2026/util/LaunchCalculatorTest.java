/*
 * LaunchCalculatorTest.java
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

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.team5924.frc2026.testutil.RobotTestBase;

class LaunchCalculatorTest extends RobotTestBase {
  private static final double EPS = 1e-6;

  @ParameterizedTest
  @CsvSource({"2.0, 2.0", "3.0, 6.0", "1.5, 4.0", "6.0, 1.0"})
  void aimedPoseKeepsRobotTranslation(double x, double y) {
    setAlliance(Alliance.Blue);
    Translation2d robot = new Translation2d(x, y);
    assertEquals(robot, LaunchCalculator.getStationaryAimedPose(robot, false).getTranslation());
  }

  @ParameterizedTest
  @CsvSource({"2.0, 2.0", "3.0, 6.0", "1.5, 4.0", "6.0, 1.0"})
  void redAimIsBlueAimMirrored(double x, double y) {
    Translation2d blueRobot = new Translation2d(x, y);
    setAlliance(Alliance.Blue);
    Pose2d blueAim = LaunchCalculator.getStationaryAimedPose(blueRobot, false);

    setAlliance(Alliance.Red);
    Translation2d redRobot = AllianceFlipUtil.apply(blueRobot);
    Pose2d redAim = LaunchCalculator.getStationaryAimedPose(redRobot, false);

    // The same shot from the mirrored spot on the red side should face the opposite direction
    assertEquals(
        0.0,
        redAim.getRotation().minus(AllianceFlipUtil.apply(blueAim.getRotation())).getRadians(),
        0.01);
  }

  @ParameterizedTest
  @CsvSource({"2.0, 2.0", "6.0, 1.0"})
  void forceBlueIgnoresAlliance(double x, double y) {
    Translation2d robot = new Translation2d(x, y);
    setAlliance(Alliance.Blue);
    Pose2d blueAim = LaunchCalculator.getStationaryAimedPose(robot, true);

    setAlliance(Alliance.Red);
    Pose2d forcedAim = LaunchCalculator.getStationaryAimedPose(robot, true);

    assertEquals(blueAim.getRotation().getRadians(), forcedAim.getRotation().getRadians(), EPS);
  }
}
