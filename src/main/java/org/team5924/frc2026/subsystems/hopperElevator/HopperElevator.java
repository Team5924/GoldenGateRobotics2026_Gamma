/*
 * HopperElevator.java
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

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.FieldState;
import org.team5924.frc2026.util.EqualsUtil;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class HopperElevator extends SubsystemBase {
  private final HopperElevatorIO io;
  private final HopperElevatorIOInputsAutoLogged inputs = new HopperElevatorIOInputsAutoLogged();

  private final Alert motorDisconnected;

  protected final Alert overheatAlert;

  @Getter private HopperElevatorState goalState = HopperElevatorState.STOW;
  private HopperElevatorState currentState = HopperElevatorState.STOW;
  private boolean isAtSetpoint = false;
  private double lastStateChange = 0.0;
  private double timeSinceLastStateChange = 0.0;

  @Setter private double input;

  @RequiredArgsConstructor
  @Getter
  public enum HopperElevatorState {
    OFF(() -> 0.0),

    STOW(new LoggedTunableNumber("HopperElevator/StowHeightMeters", 0.0)),
    EXTENDED(new LoggedTunableNumber("HopperElevator/ExtendedHeightMeters", 0.0)),
    MANUAL(new LoggedTunableNumber("HopperElevator/ManualVolts", 0.0)),
    MOVING(() -> 0.0);

    private final DoubleSupplier heightMeters;
  }

  public HopperElevator(HopperElevatorIO io) {
    this.io = io;
    this.goalState = HopperElevatorState.STOW;

    this.motorDisconnected =
        new Alert("HopperElevator Motor Disconnected!", Alert.AlertType.kWarning);
    overheatAlert = new Alert("HopperElevator motor overheating!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.periodicUpdates();
    io.updateInputs(inputs);
    Logger.processInputs("HopperElevator", inputs);

    motorDisconnected.set(!inputs.motorConnected);
    overheatAlert.set(inputs.tempCelsius > Constants.OVERHEAT_THRESHOLD);

    handleCurrentState();

    Logger.recordOutput("HopperElevator/GoalState", goalState.toString());
    Logger.recordOutput("HopperElevator/CurrentState", currentState.toString());
    Logger.recordOutput("HopperElevator/TargetMeters", goalState.getHeightMeters().getAsDouble());
    Logger.recordOutput("HopperElevator/CurrentMeters", inputs.positionMeters);
    Logger.recordOutput("HopperElevator/IsAtSetpoint", isAtSetpoint);
    Logger.recordOutput("HopperElevator/TimeSinceLastStateChange", timeSinceLastStateChange);
  }

  /* Checks if elevator is at setpoint */
  @SuppressWarnings("unused")
  public boolean isAtSetpoint() {
    return (!Constants.HopperElevator.ENABLE_TIMEOUT
            || timeSinceLastStateChange > Constants.HopperElevator.STATE_TIMEOUT)
        && EqualsUtil.epsilonEquals(
            inputs.setpointMeters, inputs.positionMeters, Constants.HopperElevator.EPSILON_METERS);
  }

  private void handleCurrentState() {
    timeSinceLastStateChange = FieldState.getInstance().getTime() - lastStateChange;
    isAtSetpoint = isAtSetpoint();

    switch (currentState) {
      case MANUAL:
        handleManualState();
        break;
      case MOVING:
        io.setHeight(goalState.getHeightMeters().getAsDouble());
        if (isAtSetpoint) currentState = goalState;
        break;
      default:
        if (!isAtSetpoint) io.setHeight(goalState.getHeightMeters().getAsDouble());
    }
  }

  private void handleManualState() {
    if (!goalState.equals(HopperElevatorState.MANUAL)) return;

    if (Math.abs(input) <= Constants.JOYSTICK_DEADZONE) {
      io.stop();
      return;
    }

    io.runVolts(HopperElevatorState.MANUAL.getHeightMeters().getAsDouble() * input);
  }

  public void setGoalState(HopperElevatorState goalState) {
    if (this.goalState.equals(goalState)) return;
    if (goalState.equals(HopperElevatorState.MANUAL)
        && Math.abs(input) <= Constants.JOYSTICK_DEADZONE) return;
    this.goalState = goalState;
    switch (goalState) {
      case STOW, EXTENDED, MANUAL: 
        currentState = goalState;
      case MOVING:
        DriverStation.reportError(
            "HopperElevator: MOVING is an invalid goal state; it is a transition state!!", null);
        return;
      default:
        currentState = HopperElevatorState.MOVING;
        break;

    }
    lastStateChange = FieldState.getInstance().getTime();
  }
}
