/*
 * RobotTestBase.java
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

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.team5924.frc2026.MatchState;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.util.AllianceFlipUtil;

/**
 * Base class for tests that touch WPILib, the command scheduler, or our singletons.
 *
 * <p>Every test starts with a fresh HAL, a disabled driver station with no alliance, paused time,
 * an empty command scheduler, and fresh {@link RobotState} / {@link MatchState} instances. Pure
 * math tests that use none of these do not need to extend this class.
 *
 * <p>Time does not advance on its own; call {@link #stepTime(double)} to move it forward.
 */
public abstract class RobotTestBase {
  @BeforeEach
  void setUpRobotTestBase() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    SimHooks.pauseTiming();
    resetScheduler();
    RobotState.resetForTests();
    MatchState.resetForTests();
    AllianceFlipUtil.HALdisabled = false;
  }

  @AfterEach
  void tearDownRobotTestBase() {
    resetScheduler();
    SimHooks.resumeTiming();
  }

  private static void resetScheduler() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    scheduler.unregisterAllSubsystems();
    scheduler.getActiveButtonLoop().clear();
  }

  /** Advances simulated time (FPGA time, and therefore AdvantageKit's timestamp) by seconds. */
  protected static void stepTime(double seconds) {
    SimHooks.stepTiming(seconds);
  }

  /**
   * Sets the simulated driver station. This is the one place to change when WPILib 2027 replaces
   * these calls with the RobotMode enum.
   *
   * @param alliance the alliance, or null for no alliance
   */
  protected static void setDriverStation(Alliance alliance, boolean enabled, boolean autonomous) {
    if (alliance == null) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
    } else {
      DriverStationSim.setAllianceStationId(
          alliance == Alliance.Blue ? AllianceStationID.Blue1 : AllianceStationID.Red1);
    }
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.setAutonomous(autonomous);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
  }

  /** Sets the alliance with the robot disabled in teleop. */
  protected static void setAlliance(Alliance alliance) {
    setDriverStation(alliance, false, false);
  }
}
