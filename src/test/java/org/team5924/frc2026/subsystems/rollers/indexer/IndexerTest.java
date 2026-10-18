/*
 * IndexerTest.java
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

package org.team5924.frc2026.subsystems.rollers.indexer;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerIOFake;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerIOFake.Command;
import org.team5924.frc2026.subsystems.rollers.indexer.Indexer.IndexerState;
import org.team5924.frc2026.testutil.RobotTestBase;

class IndexerTest extends RobotTestBase {
  private GenericRollerIOFake io;
  private Indexer indexer;

  @BeforeEach
  void setUp() {
    io = new GenericRollerIOFake();
    indexer = new Indexer(io);
  }

  @Test
  void doesNotFeedUntilFlywheelIsReady() {
    indexer.setGoalState(IndexerState.INDEXING);
    indexer.periodic();
    assertEquals(Command.STOP, io.lastCommand);
  }

  @Test
  void feedsOnceFlywheelIsReady() {
    RobotState.getInstance().setFlywheelReady(true);
    indexer.setGoalState(IndexerState.INDEXING);
    indexer.periodic();
    assertEquals(Command.VOLTS, io.lastCommand);
    assertEquals(IndexerState.INDEXING.getVoltageSupplier().getAsDouble(), io.lastValue);
  }

  @Test
  void stopsFeedingWhenFlywheelDropsOut() {
    RobotState.getInstance().setFlywheelReady(true);
    indexer.setGoalState(IndexerState.INDEXING);
    indexer.periodic();

    RobotState.getInstance().setFlywheelReady(false);
    indexer.periodic();
    assertEquals(Command.STOP, io.lastCommand);
  }

  @Test
  void offRunsZeroVolts() {
    RobotState.getInstance().setFlywheelReady(true);
    indexer.setGoalState(IndexerState.OFF);
    indexer.periodic();
    assertEquals(Command.VOLTS, io.lastCommand);
    assertEquals(0.0, io.lastValue);
  }
}
