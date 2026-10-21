/*
 * RobotTestBaseTest.java
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

package org.team5924.frc2026.testutil;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import org.team5924.frc2026.MatchState;
import org.team5924.frc2026.RobotState;

/** Checks that the shared test setup really gives each test a clean slate. */
class RobotTestBaseTest extends RobotTestBase {
  private static RobotState previousRobotState;
  private static MatchState previousMatchState;

  @RepeatedTest(2)
  void singletonsAreFreshForEachTest() {
    // A flag set by the previous repetition must not leak into this one
    assertFalse(RobotState.getInstance().isFlywheelReady());
    RobotState.getInstance().setFlywheelReady(true);

    assertNotSame(previousRobotState, RobotState.getInstance());
    assertNotSame(previousMatchState, MatchState.getInstance());
    previousRobotState = RobotState.getInstance();
    previousMatchState = MatchState.getInstance();
  }

  @Test
  void driverStationStartsDisabledWithNoAlliance() {
    assertTrue(DriverStation.isDisabled());
    assertTrue(DriverStation.getAlliance().isEmpty());
  }

  @Test
  void setDriverStationAppliesAllianceAndMode() {
    setDriverStation(Alliance.Red, true, true);
    assertEquals(Alliance.Red, DriverStation.getAlliance().orElseThrow());
    assertTrue(DriverStation.isAutonomousEnabled());
  }

  @Test
  void timeOnlyMovesWhenStepped() {
    double start = Timer.getFPGATimestamp();
    stepTime(1.5);
    assertEquals(start + 1.5, Timer.getFPGATimestamp(), 1e-3);
  }
}
