package org.team5924.frc2026.subsystems.rollers.shooterFlywheel;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem.VoltageState;
import org.team5924.frc2026.subsystems.sensors.BeamBreakIO;
import org.team5924.frc2026.subsystems.sensors.BeamBreakIOInputsAutoLogged;
import org.team5924.frc2026.util.LoggedTunableNumber;

import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ShooterFlywheel {
    

  @RequiredArgsConstructor
  @Getter
  public enum ShooterFlywheelState implements VoltageState {
    OFF(() -> 0.0),

    // voltage speed at which to rotate the rollers
    MANUAL(new LoggedTunableNumber("ShooterFlywheel/Manual", 12.0)),

    // using a double supplier of 0.0 because these will be auto-aim-calculated values
    AUTO_SHOOTING(() -> 0.0),
    NEUTRAL_SHUFFLING(() -> 0.0),
    OPPONENT_SHUFFLING(() -> 0.0),

    // TODO: test and update volts
    BUMPER_SHOOTING(new LoggedTunableNumber("ShooterFlywheel/BumperShooting", 12.0));

    private final DoubleSupplier voltageSupplier;
  }

  private ShooterFlywheelState goalState = ShooterFlywheelState.OFF;

  // Shooter Beam Break
  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  private final boolean isLeft;

  @Setter private double input;

  public ShooterFlywheel(ShooterFlywheelIO io, BeamBreakIO beamBreakIO, boolean isLeft) {
    super("ShooterFlywheel", io);
    this.beamBreakIO = beamBreakIO;
    this.isLeft = isLeft;
  }

  @Override
  protected void handleCurrentState() {
    switch (goalState) {
      case OFF:
        io.runVolts(0);
        break;

      case MANUAL:
        io.runVolts(getGoalState().getVoltageSupplier().getAsDouble() * input);
        break;

      case AUTO_SHOOTING:
        // TODO: implement
        break;

      case NEUTRAL_SHUFFLING:
        // TODO: implement
        break;

      case OPPONENT_SHUFFLING:
        // TODO: implement
        break;

      case BUMPER_SHOOTING:
        super.handleCurrentState();
        break;

      default:
        DriverStation.reportWarning(
            "Shooter Roller: " + goalState.name() + " is not a state", null);
        break;
    }
  }

  public void setGoalState(ShooterFlywheelState goalState) {
    this.goalState = goalState;

    if (isLeft) RobotState.getInstance().setLeftShooterFlywheelState(goalState);
    else RobotState.getInstance().setRightShooterFlywheelState(goalState);
  }

  @Override
  public void periodic() {
    super.periodic();
    beamBreakIO.updateInputs(beamBreakInputs);
    Logger.processInputs("ShooterFlywheel/BeamBreak", beamBreakInputs);
  }
}
