/*
 * IntakeFollowerIOTalonFX.java
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

package org.team5924.frc2026.subsystems.rollers.intake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import org.team5924.frc2026.Constants.Intake;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerIOTalonFX;

public class IntakeFollowerIOTalonFX extends GenericRollerIOTalonFX implements IntakeFollowerIO {
  public IntakeFollowerIOTalonFX() {
    super(Intake.FOLLOWER_CAN_ID, Intake.BUS, Intake.CONFIG, Intake.MOTOR_TO_MECHANISM);

    talon.setControl(new Follower(Intake.CAN_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void runVolts(double volts) {}

  @Override
  public void stop() {}
}
