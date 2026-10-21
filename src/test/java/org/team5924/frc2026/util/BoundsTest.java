/*
 * BoundsTest.java
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

package org.team5924.frc2026.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class BoundsTest {
  private final Bounds bounds = new Bounds(1.0, 3.0, -2.0, 2.0);

  @ParameterizedTest
  @CsvSource({"2.0, 0.0", "1.0, -2.0", "3.0, 2.0", "1.0, 2.0"})
  void containsPointsInsideAndOnEdges(double x, double y) {
    assertTrue(bounds.contains(new Translation2d(x, y)));
  }

  @ParameterizedTest
  @CsvSource({"0.99, 0.0", "3.01, 0.0", "2.0, -2.01", "2.0, 2.01"})
  void excludesPointsOutside(double x, double y) {
    assertFalse(bounds.contains(new Translation2d(x, y)));
  }

  @Test
  void clampLeavesInsidePointAlone() {
    Translation2d inside = new Translation2d(2.0, 1.0);
    assertEquals(inside, bounds.clamp(inside));
  }

  @Test
  void clampMovesOutsidePointToNearestEdge() {
    assertEquals(new Translation2d(3.0, -2.0), bounds.clamp(new Translation2d(10.0, -10.0)));
    assertEquals(new Translation2d(1.0, 2.0), bounds.clamp(new Translation2d(-5.0, 5.0)));
  }
}
