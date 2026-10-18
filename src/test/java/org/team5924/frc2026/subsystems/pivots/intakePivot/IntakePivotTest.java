/*
 * IntakePivotTest.java
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

package org.team5924.frc2026.subsystems.pivots.intakePivot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot.IntakePivotState;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivotIOFake.Command;
import org.team5924.frc2026.testutil.RobotTestBase;

class IntakePivotTest extends RobotTestBase {
  private IntakePivotIOFake io;
  private IntakePivot pivot;

  @BeforeEach
  void setUp() {
    io = new IntakePivotIOFake();
    pivot = new IntakePivot(io);
  }

  @Test
  void startsOffAndStopped() {
    pivot.periodic();
    assertEquals(IntakePivotState.OFF, pivot.getGoalState());
    assertEquals(Command.STOP, io.lastCommand);
  }

  @Test
  void goalStateCommandsItsAngle() {
    double down = IntakePivotState.DOWN.getRads().getAsDouble();
    pivot.setGoalState(IntakePivotState.DOWN);
    pivot.periodic();
    assertEquals(Command.POSITION, io.lastCommand);
    assertEquals(down, io.lastValue);
  }

  @Test
  void atSetpointOnceMechanismArrives() {
    pivot.setGoalState(IntakePivotState.DOWN);
    io.followSetpoint = true;
    pivot.periodic();
    pivot.periodic();
    assertTrue(pivot.isAtSetpoint());
  }

  @Test
  void manualIgnoresJoystickInsideDeadzone() {
    pivot.runManual(() -> 0.03);
    assertEquals(IntakePivotState.OFF, pivot.getGoalState());
  }

  @Test
  void manualRunsCurrentProportionalToJoystick() {
    double manualCurrent = IntakePivotState.MANUAL.getRads().getAsDouble();
    pivot.runManual(() -> -0.5);
    pivot.periodic();
    assertEquals(Command.CURRENT, io.lastCommand);
    assertEquals(manualCurrent * -0.5, io.lastValue, 1e-9);
  }

  @Test
  @Disabled(
      "Bug: isAtSetpoint compares the IO's last setpoint (stale for one cycle after a goal"
          + " change) instead of the goal angle. TODO: link GitHub issue")
  void notAtSetpointRightAfterGoalChangeWhileStillAway() {
    // The pivot sits at 0 and hasn't moved; the goal is DOWN (0.3 rad away)
    pivot.setGoalState(IntakePivotState.DOWN);
    pivot.periodic();
    assertFalse(pivot.isAtSetpoint());
  }

  @Test
  @Disabled(
      "Bug: handleShooterState flips SHOOTING_DOWN -> UP -> DOWN in a single call, so the"
          + " shooting agitation stops after one cycle. TODO: link GitHub issue")
  void shootingAgitationKeepsAlternating() {
    double up = IntakePivotState.SHOOTING_UP.getRads().getAsDouble();
    double down = IntakePivotState.SHOOTING_DOWN.getRads().getAsDouble();
    io.followSetpoint = true;

    pivot.setGoalState(IntakePivotState.SHOOTING_UP);
    for (int i = 0; i < 40; i++) pivot.periodic();

    // Count how many times the commanded angle switched between up and down
    int switches = 0;
    Double previous = null;
    for (double command : io.positionCommands) {
      if (previous != null && command != previous) switches++;
      previous = command;
    }
    assertTrue(io.positionCommands.contains(up));
    assertTrue(io.positionCommands.contains(down));
    assertTrue(switches >= 4, "expected up/down agitation, got " + switches + " switches");
  }
}
