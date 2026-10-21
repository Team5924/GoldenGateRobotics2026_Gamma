/*
 * FlywheelIOFake.java
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

/** Test double for {@link FlywheelIO}: records the last command and reports scripted readings. */
public class FlywheelIOFake implements FlywheelIO {
  public enum Command {
    NONE,
    VOLTS,
    VELOCITY,
    STOP
  }

  /** The last command the subsystem sent. */
  public Command lastCommand = Command.NONE;

  /** The value sent with the last command (volts or rotations/sec). */
  public double lastValue = 0.0;

  /** Velocity reported on the next updateInputs call; set this to simulate the wheel spinning. */
  public double velocityRotationsPerSec = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRotationsPerSec = velocityRotationsPerSec;
    inputs.setpointVelocityRotationsPerSec = lastCommand == Command.VELOCITY ? lastValue : 0.0;
  }

  @Override
  public void runVolts(double volts) {
    lastCommand = Command.VOLTS;
    lastValue = volts;
  }

  @Override
  public void setVelocity(double velocity) {
    lastCommand = Command.VELOCITY;
    lastValue = velocity;
  }

  @Override
  public void stop() {
    lastCommand = Command.STOP;
    lastValue = 0.0;
  }
}
