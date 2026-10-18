/*
 * IntakePivotIOSimTest.java
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

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivotIO.IntakePivotIOInputs;
import org.team5924.frc2026.testutil.RobotTestBase;

/** Checks that the simulated pivot reports what a real one would, so sim behavior is honest. */
class IntakePivotIOSimTest extends RobotTestBase {
  @Test
  @Disabled(
      "Bug: updateInputs never sets setpointRads (always 0) and overwrites positionRads with the"
          + " setpoint. TODO: link GitHub issue")
  void reportsCommandedSetpoint() {
    IntakePivotIOSim sim = new IntakePivotIOSim();
    IntakePivotIOInputs inputs = new IntakePivotIOInputs();

    sim.setPosition(1.0);
    sim.updateInputs(inputs);

    assertEquals(1.0, inputs.setpointRads, 1e-9);
  }
}
