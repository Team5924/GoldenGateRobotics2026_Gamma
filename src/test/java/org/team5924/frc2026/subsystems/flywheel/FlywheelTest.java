/*
 * FlywheelTest.java
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

package org.team5924.frc2026.subsystems.flywheel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.flywheel.Flywheel.FlywheelState;
import org.team5924.frc2026.subsystems.flywheel.FlywheelIOFake.Command;
import org.team5924.frc2026.testutil.RobotTestBase;

class FlywheelTest extends RobotTestBase {
  private FlywheelIOFake io;
  private Flywheel flywheel;

  @BeforeEach
  void setUp() {
    io = new FlywheelIOFake();
    flywheel = new Flywheel(io);
  }

  private static boolean flywheelReady() {
    return RobotState.getInstance().isFlywheelReady();
  }

  @Test
  void startsOffAndStopped() {
    flywheel.periodic();
    assertEquals(FlywheelState.OFF, flywheel.getGoalState());
    assertEquals(Command.STOP, io.lastCommand);
    assertFalse(flywheelReady());
  }

  @Test
  void launchSpinsUpAndReportsReadyAtSpeed() {
    double target = FlywheelState.FAST_LAUNCH.getVelocityRotationsPerSec().getAsDouble();
    flywheel.setGoalState(FlywheelState.FAST_LAUNCH);

    flywheel.periodic();
    assertEquals(Command.VELOCITY, io.lastCommand);
    assertEquals(target, io.lastValue);
    assertFalse(flywheelReady(), "not ready while the wheel is still stopped");

    io.velocityRotationsPerSec = target;
    flywheel.periodic();
    assertTrue(flywheel.isAtSetpoint());
    assertTrue(flywheelReady());

    // Keeps holding the target once settled
    flywheel.periodic();
    assertEquals(Command.VELOCITY, io.lastCommand);
    assertEquals(target, io.lastValue);
  }

  @Test
  void fasterThanTargetCountsAsReady() {
    flywheel.setGoalState(FlywheelState.FAST_LAUNCH);
    io.velocityRotationsPerSec = 100.0;
    flywheel.periodic();
    assertTrue(flywheelReady());
  }

  @Test
  void bumpStatesRunOpenLoopVolts() {
    flywheel.setGoalState(FlywheelState.B8);
    flywheel.periodic();
    assertEquals(Command.VOLTS, io.lastCommand);
    assertEquals(8.0, io.lastValue);
    assertTrue(flywheelReady(), "bump states are always considered at setpoint");
  }

  @Test
  void offIsNeverReadyEvenWhileSpinning() {
    io.velocityRotationsPerSec = 30.0;
    flywheel.setGoalState(FlywheelState.OFF);
    flywheel.periodic();
    assertFalse(flywheelReady());
  }

  @Test
  void idleIsBelowEpsilonSoMotorStopsAndIsNotReady() {
    flywheel.setGoalState(FlywheelState.IDLE);
    flywheel.periodic();
    assertEquals(Command.STOP, io.lastCommand);
    assertFalse(flywheelReady());
  }

  @Test
  void manualSetpointIsClampedToValidRange() {
    flywheel.setAutoInput(150.0);
    flywheel.setGoalState(FlywheelState.MANUAL_SETPOINT);
    flywheel.periodic();
    assertEquals(Command.VELOCITY, io.lastCommand);
    assertEquals(100.0, io.lastValue);

    flywheel.setAutoInput(-20.0);
    flywheel.periodic();
    assertEquals(Command.STOP, io.lastCommand, "negative input clamps to 0, which stops");
  }
}
