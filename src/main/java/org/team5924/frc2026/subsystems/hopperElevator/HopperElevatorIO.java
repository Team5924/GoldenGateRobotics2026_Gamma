/*
 * HopperElevatorIO.java
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

package org.team5924.frc2026.subsystems.hopperElevator;

import lombok.Getter;
import org.littletonrobotics.junction.AutoLog;

public interface HopperElevatorIO {
  @AutoLog
  @Getter
  public static class HopperElevatorIOInputs {
    public boolean motorConnected = true;

    public double position = 0.0;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    public double positionMeters = 0.0;
    public double velMetersPerSecond = 0.0;

    public double motionMagicVelocityTarget = 0.0;
    public double motionMagicPositionTarget = 0.0;

    public double setpointMeters = 0.0;

    public double acceleration = 0.0;

    public boolean cancoderConnected = true;
    public double cancoderAbsolutePosition = 0.0;
    public double cancoderVelocity = 0.0;
    public double cancoderSupplyVoltage = 0.0;
    public double cancoderPositionRotations = 0.0;
  }

  public default void updateInputs(HopperElevatorIOInputs inputs) {}

  public default void periodicUpdates() {}

  public default void runVolts(double volts) {}

  public default void setHeight(double heightMeters) {}

  public default void stop() {}
}
