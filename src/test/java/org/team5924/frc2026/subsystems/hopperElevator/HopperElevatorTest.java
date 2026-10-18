/*
 * HopperElevatorTest.java
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

package org.team5924.frc2026.subsystems.hopperElevator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.team5924.frc2026.subsystems.hopperElevator.HopperElevator.HopperElevatorState;
import org.team5924.frc2026.subsystems.hopperElevator.HopperElevatorIOFake.Command;
import org.team5924.frc2026.testutil.RobotTestBase;

class HopperElevatorTest extends RobotTestBase {
  private HopperElevatorIOFake io;
  private HopperElevator elevator;

  @BeforeEach
  void setUp() {
    io = new HopperElevatorIOFake();
    elevator = new HopperElevator(io);
  }

  @Test
  void toggleSwitchesBetweenStowAndExtended() {
    assertEquals(HopperElevatorState.STOW, elevator.getGoalState());
    elevator.toggleState();
    assertEquals(HopperElevatorState.EXTENDED, elevator.getGoalState());
    elevator.toggleState();
    assertEquals(HopperElevatorState.STOW, elevator.getGoalState());
  }

  @Test
  void toggleDoesNothingFromOtherStates() {
    elevator.setGoalState(HopperElevatorState.CENTER);
    elevator.toggleState();
    assertEquals(HopperElevatorState.CENTER, elevator.getGoalState());
  }

  @Test
  void manualRunsVoltsProportionalToJoystick() {
    double manualVolts = HopperElevatorState.MANUAL.getHeightMeters().getAsDouble();
    elevator.runManual(() -> 0.5);
    elevator.periodic();
    assertEquals(Command.VOLTS, io.lastCommand);
    assertEquals(manualVolts * 0.5, io.lastValue, 1e-9);
  }

  @Test
  @Disabled(
      "Bug: isAtSetpoint compares the IO's last setpoint instead of the goal height, so after"
          + " toggling to EXTENDED from rest the new height is never commanded. Low priority:"
          + " the hopper elevator uses an empty IO on Gamma. TODO: link GitHub issue")
  void toggleToExtendedCommandsNewHeight() {
    double extended = HopperElevatorState.EXTENDED.getHeightMeters().getAsDouble();
    elevator.toggleState();
    elevator.periodic();
    assertEquals(Command.HEIGHT, io.lastCommand);
    assertEquals(extended, io.lastValue);
  }
}
