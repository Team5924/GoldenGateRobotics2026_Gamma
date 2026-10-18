/*
 * ShooterHoodTest.java
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

package org.team5924.frc2026.subsystems.pivots.shooterHood;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHood.ShooterHoodState;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHoodIOFake.Command;
import org.team5924.frc2026.testutil.RobotTestBase;

class ShooterHoodTest extends RobotTestBase {
  private ShooterHoodIOFake io;
  private ShooterHood hood;

  @BeforeEach
  void setUp() {
    io = new ShooterHoodIOFake();
    hood = new ShooterHood(io);
  }

  @Test
  void startsOffAndStopped() {
    hood.periodic();
    assertEquals(ShooterHoodState.OFF, hood.getGoalState());
    assertEquals(Command.STOP, io.lastCommand);
  }

  @Test
  void goalStateCommandsItsAngle() {
    double center = ShooterHoodState.CENTER.getRads().getAsDouble();
    hood.setGoalState(ShooterHoodState.CENTER);
    hood.periodic();
    assertEquals(Command.POSITION, io.lastCommand);
    assertEquals(center, io.lastValue);
  }

  @Test
  void atSetpointOnlyWithinTolerance() {
    double center = ShooterHoodState.CENTER.getRads().getAsDouble();
    hood.setGoalState(ShooterHoodState.CENTER);

    io.positionRads = center + Math.toRadians(0.2);
    hood.periodic();
    assertTrue(hood.isAtSetpoint());

    io.positionRads = center + Math.toRadians(1.0);
    hood.periodic();
    assertFalse(hood.isAtSetpoint());
  }

  @Test
  void manualIgnoresJoystickInsideDeadzone() {
    hood.runManual(() -> 0.03);
    assertEquals(ShooterHoodState.OFF, hood.getGoalState());
  }

  @Test
  void manualRunsCurrentProportionalToJoystick() {
    double manualCurrent = ShooterHoodState.MANUAL.getRads().getAsDouble();
    hood.runManual(() -> 0.5);
    hood.periodic();
    assertEquals(ShooterHoodState.MANUAL, hood.getGoalState());
    assertEquals(Command.CURRENT, io.lastCommand);
    assertEquals(manualCurrent * 0.5, io.lastValue, 1e-9);

    // Releasing the stick stops the motor
    hood.setInput(0.0);
    hood.periodic();
    assertEquals(Command.STOP, io.lastCommand);
  }
}
