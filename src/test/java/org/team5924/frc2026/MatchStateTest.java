/*
 * MatchStateTest.java
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

package org.team5924.frc2026;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.team5924.frc2026.MatchState.MatchShift;
import org.team5924.frc2026.testutil.RobotTestBase;

class MatchStateTest extends RobotTestBase {
  private final MatchState matchState = MatchState.getInstance();

  /** Teleop enabled on the given alliance, with the given match time and game message. */
  private static void teleop(Alliance alliance, double matchTime, String gameMessage) {
    DriverStationSim.setMatchTime(matchTime);
    DriverStationSim.setGameSpecificMessage(gameMessage);
    setDriverStation(alliance, true, false);
  }

  @ParameterizedTest(name = "{0}s left -> {1}")
  @CsvSource({
    "140, TRANSITION",
    "130.01, TRANSITION",
    "130, SHIFT_ONE",
    "106, SHIFT_ONE",
    "105, SHIFT_TWO",
    "80, SHIFT_THREE",
    "55, SHIFT_FOUR",
    "30, END_GAME",
    "1, END_GAME",
    "0, INVALID"
  })
  void teleopMatchTimeSelectsShift(double matchTime, MatchShift expected) {
    teleop(Alliance.Blue, matchTime, "");
    matchState.updateCurrentMatchShift();
    assertEquals(expected, matchState.getCurrentMatchShift());
  }

  @Test
  void autonomousIsAutoShift() {
    DriverStationSim.setMatchTime(15);
    setDriverStation(Alliance.Blue, true, true);
    matchState.updateCurrentMatchShift();
    assertEquals(MatchShift.AUTO, matchState.getCurrentMatchShift());
  }

  @Test
  void autoWinnerNeedsAlliance() {
    teleop(null, 120, "B");
    assertFalse(matchState.updateAlliance());
    assertFalse(matchState.updateIsAutoWinner());
  }

  @ParameterizedTest
  @CsvSource({"B, true", "R, true", "'', false", "X, false"})
  void autoWinnerParsesGameMessage(String message, boolean expectedValid) {
    teleop(Alliance.Blue, 120, message);
    assertTrue(matchState.updateAlliance());
    assertEquals(expectedValid, matchState.updateIsAutoWinner());
  }

  @ParameterizedTest(name = "{0} alliance, {1} won auto, {2}s left -> hub active {3}")
  @CsvSource({
    // The alliance that won auto has an inactive hub in shifts one and three
    "Blue, B, 120, false",
    "Blue, B, 90, true",
    "Blue, B, 70, false",
    "Blue, B, 40, true",
    "Red, B, 120, true",
    "Red, B, 90, false",
    // Transition and end game are always active
    "Blue, B, 135, true",
    "Blue, B, 20, true",
    // Missing game data: assume active
    "Blue, '', 120, true"
  })
  void hubActiveAlternatesByShift(
      Alliance alliance, String winner, double matchTime, boolean expectedActive) {
    teleop(alliance, matchTime, winner);
    matchState.updateCurrentMatchShift();
    assertEquals(expectedActive, matchState.isHubActive());
  }
}
