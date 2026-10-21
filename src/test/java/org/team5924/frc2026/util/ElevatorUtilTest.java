/*
 * ElevatorUtilTest.java
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
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class ElevatorUtilTest {
  @ParameterizedTest
  @ValueSource(doubles = {0.0, 0.05, 0.1016, 0.2032, -0.01})
  void metersToRotationsRoundTrips(double meters) {
    double rotations = ElevatorUtil.metersToRotations(meters);
    assertEquals(meters, ElevatorUtil.rotationsToMeters(rotations), 1e-9);
  }

  @Test
  void higherMeansMoreRotations() {
    assertTrue(ElevatorUtil.metersToRotations(0.2) > ElevatorUtil.metersToRotations(0.1));
    assertEquals(0.0, ElevatorUtil.metersToRotations(0.0), 1e-12);
  }
}
