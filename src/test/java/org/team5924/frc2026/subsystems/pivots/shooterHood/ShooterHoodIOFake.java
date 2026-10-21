/*
 * ShooterHoodIOFake.java
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

/**
 * Test double for {@link ShooterHoodIO}. Like the real motor, it reports the last commanded
 * position as {@code setpointRads}.
 */
public class ShooterHoodIOFake implements ShooterHoodIO {
  public enum Command {
    NONE,
    POSITION,
    CURRENT,
    STOP
  }

  public Command lastCommand = Command.NONE;
  public double lastValue = 0.0;

  /** Position reported on the next updateInputs call. */
  public double positionRads = 0.0;

  private double setpointRads = 0.0;

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    inputs.positionRads = positionRads;
    inputs.setpointRads = setpointRads;
  }

  @Override
  public void setPosition(double rads) {
    lastCommand = Command.POSITION;
    lastValue = rads;
    setpointRads = rads;
  }

  @Override
  public void runCurrent(double current) {
    lastCommand = Command.CURRENT;
    lastValue = current;
  }

  @Override
  public void stop() {
    lastCommand = Command.STOP;
    lastValue = 0.0;
  }
}
