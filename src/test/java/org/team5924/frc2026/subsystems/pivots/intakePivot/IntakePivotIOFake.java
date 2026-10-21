/*
 * IntakePivotIOFake.java
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

import java.util.ArrayList;
import java.util.List;

/**
 * Test double for {@link IntakePivotIO}. Like the real motor, it reports the last commanded
 * position as {@code setpointRads}.
 */
public class IntakePivotIOFake implements IntakePivotIO {
  public enum Command {
    NONE,
    POSITION,
    CURRENT,
    STOP
  }

  public Command lastCommand = Command.NONE;
  public double lastValue = 0.0;

  /** Every position sent with setPosition, in order. */
  public final List<Double> positionCommands = new ArrayList<>();

  /** Position reported on the next updateInputs call. */
  public double positionRads = 0.0;

  /** If true, the pivot instantly reaches every commanded position (a perfect mechanism). */
  public boolean followSetpoint = false;

  private double setpointRads = 0.0;

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    if (followSetpoint) positionRads = setpointRads;
    inputs.positionRads = positionRads;
    inputs.setpointRads = setpointRads;
  }

  @Override
  public void setPosition(double rads) {
    lastCommand = Command.POSITION;
    lastValue = rads;
    setpointRads = rads;
    positionCommands.add(rads);
  }

  @Override
  public void runCurrent(double amps) {
    lastCommand = Command.CURRENT;
    lastValue = amps;
  }

  @Override
  public void stop() {
    lastCommand = Command.STOP;
    lastValue = 0.0;
  }
}
