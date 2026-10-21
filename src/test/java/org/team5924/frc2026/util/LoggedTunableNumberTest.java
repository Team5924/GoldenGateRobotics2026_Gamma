/*
 * LoggedTunableNumberTest.java
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

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.team5924.frc2026.testutil.RobotTestBase;

class LoggedTunableNumberTest extends RobotTestBase {
  @Test
  void returnsDefaultValue() {
    assertEquals(4.5, new LoggedTunableNumber("Test/Default", 4.5).get());
  }

  @Test
  void withoutDefaultReturnsZero() {
    assertEquals(0.0, new LoggedTunableNumber("Test/NoDefault").get());
  }

  @Test
  void defaultCanOnlyBeSetOnce() {
    LoggedTunableNumber number = new LoggedTunableNumber("Test/Once", 1.0);
    number.initDefault(2.0);
    assertEquals(1.0, number.get());
  }

  @Test
  void hasChangedIsTrackedPerCaller() {
    LoggedTunableNumber number = new LoggedTunableNumber("Test/HasChanged", 1.0);

    assertTrue(number.hasChanged(1), "first check by a caller always reports a change");
    assertFalse(number.hasChanged(1), "unchanged value should not report again");
    assertTrue(number.hasChanged(2), "a different caller gets its own first check");
  }

  @Test
  void ifChangedRunsOnlyOnChange() {
    LoggedTunableNumber number = new LoggedTunableNumber("Test/IfChanged", 3.0);
    AtomicInteger calls = new AtomicInteger();

    LoggedTunableNumber.ifChanged(7, values -> calls.incrementAndGet(), number);
    LoggedTunableNumber.ifChanged(7, values -> calls.incrementAndGet(), number);

    assertEquals(1, calls.get());
  }
}
