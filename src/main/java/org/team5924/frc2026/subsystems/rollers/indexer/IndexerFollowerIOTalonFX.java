/*
 * IndexerFollowerIOTalonFX.java
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

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import org.team5924.frc2026.Constants.Indexer;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerIOTalonFX;

public class IndexerFollowerIOTalonFX extends GenericRollerIOTalonFX implements IndexerIO {
  public IndexerFollowerIOTalonFX() {
    super(Indexer.FOLLOWER_CAN_ID, Indexer.BUS, Indexer.CONFIG, Indexer.MOTOR_TO_MECHANISM);

    talon.setControl(new Follower(Indexer.CAN_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void runVolts(double volts) {}

  @Override
  public void stop() {}
}
