package org.team5924.frc2026.subsystems.rollers.shooterRoller;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterRollerIO {
  @AutoLog
  public static class ShooterRollerIOInputs {
    public boolean shooterRollerMotorConnected = true;
    public double shooterRollerPosition = 0.0;
    public double shooterRollerPositionRads = 0.0;
    public double shooterRollerPositionCancoder = 0.0;
    public double shooterRollerVelocityRadsPerSec = 0.0;
    public double shooterRollerAppliedVoltage = 0.0;
    public double shooterRollerSupplyCurrentAmps = 0.0;
    public double shooterRollerTorqueCurrentAmps = 0.0;
    public double shooterRollerTempCelsius = 0.0;

    public double motionMagicVelocityTarget = 0.0;
    public double motionMagicPositionTarget = 0.0;

    public double setpointRads = 0.0;
    public double acceleration = 0.0;
  }

  /**
   * Updates the inputs object with the latest data from hardware
   *
   * @param inputs Inputs to update
   */
  public default void updateInputs(ShooterRollerIOInputs inputs) {}

  /** Updates that are be called in shooterRoller periodic */
  public default void periodicUpdates() {}

  /**
   * Sets the shooterRoller motor to the specified voltage
   *
   * @param volts number of volts
   */
  public default void runVolts(double volts) {}

  /**
   * Sets the shooterRoller motor to a specified angle
   *
   * @param rads target angle
   */
  public default void setPosition(double rads) {}

  /** Holds the shooterRoller motor at a set position */
  public default void holdPosition(double rads) {}

  /** stops the motor */
  default void stop() {}

  /** Sets the shooterRoller position to specified rads from center */
  default void setPositionSetpoint(double radiansFromCenter, double radsPerSecond) {}
}
