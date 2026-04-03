/*
 * AutoBuilder.java
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

package org.team5924.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.team5924.frc2026.RobotContainer;
import org.team5924.frc2026.subsystems.drive.Drive;
import org.team5924.frc2026.subsystems.flywheel.Flywheel;
import org.team5924.frc2026.subsystems.flywheel.Flywheel.FlywheelState;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot.IntakePivotState;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHood;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHood.ShooterHoodState;
import org.team5924.frc2026.subsystems.rollers.hopper.Hopper;
import org.team5924.frc2026.subsystems.rollers.indexer.Indexer;
import org.team5924.frc2026.subsystems.rollers.intake.Intake;
import org.team5924.frc2026.subsystems.rollers.intake.Intake.IntakeState;

@RequiredArgsConstructor
public class AutoBuilder {
  private final Drive drive;
  private final ShooterHood shooterHood;
  private final Flywheel flywheel;
  // private final Climb climb;
  private final Intake intake;
  private final IntakePivot intakePivot;
  private final Hopper hopper;
  private final Indexer indexer;

  // Left, Mid, Right 1-5
  private static Supplier<String> startingPositionSupplier;

  public static void setStartingPosition(Supplier<String> supplier) {
    if (supplier == null)
      throw new IllegalArgumentException("startingPositionSupplier can't be null");
    startingPositionSupplier = supplier;
  }

  public Command scoreAndClimbAuto() {
    if (startingPositionSupplier == null) {
      throw new IllegalStateException(
          "starting position must be set before building auto commands");
    }
    return Commands.defer(
        () ->
            Commands.sequence(
                startToHub(startingPositionSupplier.get()),
                shooterOn(1.0), // TODO: Edit timeout
                shooterOff(),
                RobotContainer.autoFactory.trajectoryCmd("HubToClimb")
                // Commands.run(() -> climb.setGoalState(ClimbState.L1_CLIMB), climb)
                ),
        Set.of(drive, shooterHood, flywheel));
  }

  public Command scorePickupAndClimbAuto() {
    if (startingPositionSupplier.get() == null) {
      throw new IllegalStateException(
          "starting position must be set before building auto commands");
    }
    return Commands.defer(
        () ->
            Commands.sequence(
                startToHub(startingPositionSupplier.get()),
                shooterOn(1.0), // TODO: edit timeouts
                shooterOff(),
                RobotContainer.autoFactory.trajectoryCmd("HubToDepot"),
                intakeSequence(),
                RobotContainer.autoFactory.trajectoryCmd("DepotToHub"),
                shooterOn(1.0),
                shooterOff(),
                RobotContainer.autoFactory.trajectoryCmd("HubToClimb")
                // Commands.run(() -> climb.setGoalState(ClimbState.L1_CLIMB), climb)
                ),
        Set.of(drive, shooterHood, flywheel, intake));
  }

  public Command rightDoubleSwipe() {
    return Commands.defer(
        () -> {
          if(!"Right".equals(startingPositionSupplier.get())) {
            throw new IllegalStateException("starting position must be right for this auto command");
          }
          return Commands.sequence(
                RobotContainer.autoFactory.resetOdometry("Swipe1Right"),
                Commands.deadline(
                    RobotContainer.autoFactory.trajectoryCmd("Swipe1Right"), 
                    intake()),
                intakeOff(),
                shooterOn(1.0), // TODO: Edit Timeout values
                shooterOff(),
                Commands.deadline(
                    RobotContainer.autoFactory.trajectoryCmd("Swipe2Right"), 
                    intake()),
                intakeOff(),
                shooterOn(1.0), // TODO: Edit Timeout values
                shooterOff(),
                RobotContainer.autoFactory.trajectoryCmd("Stow"));
        },
        Set.of(drive, shooterHood, flywheel, intake, intakePivot));
  }

  private Command startToHub(String startingPosition) {
    if ("Mid".equals(startingPosition)) {
      return Commands.none();
    } else {
      return Commands.sequence(
          RobotContainer.autoFactory.resetOdometry(startingPosition + "StartToHub"),
          RobotContainer.autoFactory.trajectoryCmd(startingPosition + "StartToHub"));
    }
  }

  private Command shooterOn(double timeout) {
    return Commands.parallel(
            Commands.run(() -> shooterHood.setGoalState(ShooterHoodState.AUTO), shooterHood),
            Commands.run(() -> flywheel.setGoalState(FlywheelState.AUTO), flywheel))
        .withTimeout(timeout);
  }

  private Command shooterOff() {
    return Commands.parallel(
        Commands.runOnce(() -> shooterHood.setGoalState(ShooterHoodState.OFF), shooterHood),
        Commands.runOnce(() -> flywheel.setGoalState(FlywheelState.OFF), flywheel));
  }

  private Command intakeSequence() {
    return Commands.sequence(
        Commands.deadline(RobotContainer.autoFactory.trajectoryCmd("DepotIntake"), intake()),
        intakeOff());
  }

  private Command intake() {
    return Commands.parallel(
        Commands.runOnce(() -> intakePivot.setGoalState(IntakePivotState.DOWN), intakePivot),
        Commands.run(() -> intake.setGoalState(IntakeState.INTAKE), intake));
  }

  private Command intakeOff() {
    return Commands.parallel(
        Commands.runOnce(() -> intakePivot.setGoalState(IntakePivotState.SHOOTING), intakePivot),
        Commands.runOnce(() -> intake.setGoalState(IntakeState.OFF), intake));
  }
}
