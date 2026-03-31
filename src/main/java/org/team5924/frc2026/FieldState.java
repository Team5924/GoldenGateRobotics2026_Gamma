/*
 * FieldState.java
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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class FieldState {
  private static FieldState instance = new FieldState();

  public static synchronized FieldState getInstance() {
    return instance;
  }

  /** returns the current time in seconds (counting up) */
  public double getTime() {
    return ((double) Logger.getTimestamp() * 1.0E-6);
  }

  public enum MatchShift {
    AUTO,
    TRANSITION,
    SHIFT_ONE,
    SHIFT_TWO,
    SHIFT_THREE,
    SHIFT_FOUR,
    END_GAME,

    NONE, // default state on startup
    INVALID // use this state if any info from FMS is invalid
  }

  private final double[] matchShiftTimes = {
    9999, // auto is not during teleop, so arbitrarily high time!!
    130, 105, 80, 55, 30, 0
  };

  @Getter private volatile MatchShift currentMatchShift = MatchShift.NONE;

  private boolean isAutoWinner = false; // is robot alliance the alliance winner
  private boolean hasAutoWinner = true;

  private boolean isBlue = true;
  private boolean hasAlliance = true;

  private MatchShift calculateCurrentMatchShift() {
    double time = DriverStation.getMatchTime();

    if (DriverStation.isAutonomousEnabled()) {
      return MatchShift.AUTO;
    }

    for (int i = 1; i < matchShiftTimes.length; ++i) {
      if (time > matchShiftTimes[i]) return MatchShift.values()[i];
    }

    return MatchShift.INVALID;
  }

  public void updateCurrentMatchShift() {
    currentMatchShift = calculateCurrentMatchShift();
  }

  public void logData() {
    Logger.recordOutput("FieldState/isHubActive", isHubActive());
    Logger.recordOutput("FieldState/MatchTime", DriverStation.getMatchTime());
    Logger.recordOutput("FieldState/MatchShift", currentMatchShift.toString());
    Logger.recordOutput("FieldState/isAutoWinner", isAutoWinner);
    Logger.recordOutput("FieldState/hasAutoWinner", hasAutoWinner);
    Logger.recordOutput("FieldState/isBlue", isBlue);
    Logger.recordOutput("FieldState/hasAlliance", hasAlliance);
  }

  /**
   * updates alliance
   *
   * @return hasAlliance
   */
  public boolean updateAlliance() {
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
    if (optionalAlliance.isEmpty()) {
      return hasAlliance = false;
    }

    isBlue = optionalAlliance.get().equals(Alliance.Blue);
    return hasAlliance = true;
  }

  /**
   * updates isAutoWinner
   *
   * @return hasAutoWinner
   */
  public boolean updateIsAutoWinner() {
    if (!hasAlliance) return hasAutoWinner = false;

    String gameData = DriverStation.getGameSpecificMessage();

    hasAutoWinner = true;

    if (gameData.isEmpty()) return hasAutoWinner = false;

    switch (gameData.charAt(0)) {
      case 'R' -> {
        isAutoWinner = !isBlue;
      }
      case 'B' -> {
        isAutoWinner = isBlue;
      }
      default -> {
        return hasAutoWinner = false;
      }
    }

    return true;
  }

  public boolean isHubActive() {
    updateCurrentMatchShift();

    switch (currentMatchShift) {
      case AUTO, TRANSITION, END_GAME, NONE, INVALID -> {
        // for these states, always assume active hub
        return true;
      }
      default -> { // conditionally active states
        // if no alliance or auto winner, update
        //      -> assume hub is active in the case of persisting invalid data
        if (!hasAlliance || !hasAutoWinner) {
          if (updateAlliance()) { // try to update alliance
            // if valid -> try to update auto winner
            if (!updateIsAutoWinner()) return true; // auto winner invalid
          } else {
            return true; // alliance invalid
          }
        }

        return switch (currentMatchShift) {
          case SHIFT_ONE, SHIFT_THREE -> !isAutoWinner;
          case SHIFT_TWO, SHIFT_FOUR -> isAutoWinner;
          default -> true;
        };
      }
    }
  }
}
