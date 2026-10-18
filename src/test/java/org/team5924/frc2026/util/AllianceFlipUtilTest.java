/*
 * AllianceFlipUtilTest.java
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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;
import org.team5924.frc2026.FieldConstants;
import org.team5924.frc2026.testutil.RobotTestBase;

class AllianceFlipUtilTest extends RobotTestBase {
  private static final double EPS = 1e-9;
  private static final Pose2d POSE = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(30.0));

  @Test
  void noAllianceDoesNotFlip() {
    assertFalse(AllianceFlipUtil.shouldFlip());
    assertEquals(POSE, AllianceFlipUtil.apply(POSE));
  }

  @Test
  void blueDoesNotFlip() {
    setAlliance(Alliance.Blue);
    assertFalse(AllianceFlipUtil.shouldFlip());
    assertEquals(POSE, AllianceFlipUtil.apply(POSE));
  }

  @Test
  void redRotatesPoseAboutFieldCenter() {
    setAlliance(Alliance.Red);
    assertTrue(AllianceFlipUtil.shouldFlip());

    Pose2d flipped = AllianceFlipUtil.apply(POSE);
    assertEquals(FieldConstants.fieldLength - POSE.getX(), flipped.getX(), 0.01);
    assertEquals(FieldConstants.fieldWidth - POSE.getY(), flipped.getY(), 0.01);
    // Compare rotations by difference so the test doesn't depend on angle wrapping
    assertEquals(0.0, flipped.getRotation().minus(Rotation2d.fromDegrees(210.0)).getRadians(), EPS);
  }

  @Test
  void flippingTwiceReturnsOriginal() {
    setAlliance(Alliance.Red);
    Pose2d twice = AllianceFlipUtil.apply(AllianceFlipUtil.apply(POSE));
    assertEquals(POSE.getX(), twice.getX(), EPS);
    assertEquals(POSE.getY(), twice.getY(), EPS);
    assertEquals(POSE.getRotation().getRadians(), twice.getRotation().getRadians(), EPS);
  }

  @Test
  void disableHALStopsFlipping() {
    setAlliance(Alliance.Red);
    AllianceFlipUtil.disableHAL();
    assertFalse(AllianceFlipUtil.shouldFlip());
  }

  @Test
  void hardCodedFieldSizeMatchesFieldConstants() {
    // AllianceFlipUtil hard-codes 651.2" x 317.7"; it must agree with the AprilTag field layout
    setAlliance(Alliance.Red);
    assertEquals(FieldConstants.fieldLength, AllianceFlipUtil.applyX(0.0), 0.01);
    assertEquals(FieldConstants.fieldWidth, AllianceFlipUtil.applyY(0.0), 0.01);
  }
}
