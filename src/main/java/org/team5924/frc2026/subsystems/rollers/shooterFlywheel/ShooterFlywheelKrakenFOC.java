/*
 * ShooterFlywheelIOTalonFX.java
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

package org.team5924.frc2026.subsystems.rollers.shooterFlywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class ShooterFlywheelKrakenFOC implements ShooterFlywheelIO {
  /* Hardware */
  private final TalonFX shooterFlywheelLeaderTalon;
  private final TalonFX shooterFlywheelFollowerTalon;

  /* Configurators */
  private TalonFXConfigurator shooterFlywheelTalonLeaderConfig;
  private TalonFXConfigurator shooterFlywheelTalonFollowerConfig;

  /* Configs  */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;

  /* Gains */
  private final LoggedTunableNumber kP = new LoggedTunableNumber("ShooterFlywheel/kP", 3.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("ShooterFlywheel/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("ShooterFlywheel/kD", 0.07);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("ShooterFlywheel/kS", 0.13);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("ShooterFlywheel/kV", 0.4);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("ShooterFlywheel/kA", 0.00);

  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("ShooterFlywheel/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("ShooterFlywheel/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerk = 
      new LoggedTunableNumber("ShooterFlywheel/MotionJerk", 0.0);

  /* Status Signals */
  private final StatusSignal<Angle> shooterFlywheelPosition;
  private final StatusSignal<AngularVelocity> shooterFlywheelVelocity;
  private final StatusSignal<Voltage> shooterFlywheelAppliedVoltage;
  private final StatusSignal<Current> shooterFlywheelSupplyCurrent;
  private final StatusSignal<Current> shooterFlywheelTorqueCurrent;
  private final StatusSignal<Temperature> shooterFlywheelTempCelsius;


  private final StatusSignal<Double> closedLoopReferenceSlope;
  private double prevClosedLoopReferenceSlope = 0.0;
  private double prevReferenceSlopeTimestamp = 0.0;

  private final VoltageOut voltageOut;
  private final PositionVoltage positionOut;
  private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;

  private final double reduction;

  public ShooterFlywheelKrakenFOC(boolean isLeft) {
    shooterFlywheelLeaderTalon =
        new TalonFX(
            isLeft ? Constants.ShooterFlywheelLeaderLeft.CAN_ID : Constants.ShooterFlywheelLeaderRight.CAN_ID,
            new CANBus(isLeft ? Constants.ShooterFlywheelLeaderLeft.BUS : Constants.ShooterFlywheelLeaderRight.BUS));

    shooterFlywheelFollowerTalon =
        new TalonFX(
            isLeft ? Constants.ShooterFlywheelFollowerLeft.CAN_ID : Constants.ShooterFlywheelFollowerRight.CAN_ID,
            new CANBus(isLeft ? Constants.ShooterFlywheelFollowerLeft.BUS : Constants.ShooterFlywheelFollowerRight.BUS));

    reduction =
        isLeft ? Constants.ShooterFlywheelLeft.MOTOR_TO_MECHANISM : Constants.TurretRight.MOTOR_TO_MECHANISM;

    shooterFlywheelTalonLeaderConfig = shooterFlywheelLeaderTalon.getConfigurator();
    shooterFlywheelTalonFollowerConfig = shooterFlywheelFollowerTalon.getConfigurator();

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();

    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[8];

    statusArray[0] =
        shooterFlywheelTalonLeaderConfig.apply(
            isLeft ? Constants.ShooterFlywheelLeaderLeft.CONFIG : Constants.ShooterFlywheelLeaderRight.CONFIG);
    statusArray[1] =
        shooterFlywheelTalonLeaderConfig.apply(
            isLeft
                ? Constants.ShooterFlywheelLeaderLeft.OPEN_LOOP_RAMPS_CONFIGS
                : Constants.ShooterFlywheelLeaderRight.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[2] =
        shooterFlywheelTalonLeaderConfig.apply(
            isLeft
                ? Constants.ShooterFlywheelLeaderLeft.CLOSED_LOOP_RAMPS_CONFIGS
                : Constants.ShooterFlywheelLeaderRight.CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[3] = shooterFlywheelTalonLeaderConfig.apply(motionMagicConfigs);
    statusArray[4] = shooterFlywheelTalonLeaderConfig.apply(slot0Configs);

    statusArray[5] =
        shooterFlywheelTalonFollowerConfig.apply(
            isLeft ? Constants.ShooterFlywheelFollowerLeft.CONFIG : Constants.ShooterFlywheelFollowerRight.CONFIG);
    statusArray[6] =
        shooterFlywheelTalonFollowerConfig.apply(
            isLeft
                ? Constants.ShooterFlywheelFollowerLeft.OPEN_LOOP_RAMPS_CONFIGS
                : Constants.ShooterFlywheelFollowerRight.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[7] =
        shooterFlywheelTalonFollowerConfig.apply(
            isLeft
                ? Constants.ShooterFlywheelFollowerLeft.CLOSED_LOOP_RAMPS_CONFIGS
                : Constants.ShooterFlywheelFollowerRight.CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[8] = shooterFlywheelTalonFollowerConfig.apply(motionMagicConfigs);
    statusArray[9] = shooterFlywheelTalonFollowerConfig.apply(slot0Configs);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING, "ShooterFlywheel Configs", "Error in shooterFlywheel configs!"));

    Logger.recordOutput("ShooterFlywheel/InitConfReport", statusArray);

    // Get select status signals and set update frequency
    shooterFlywheelPosition = shooterFlywheelLeaderTalon.getPosition();
    shooterFlywheelVelocity = shooterFlywheelLeaderTalon.getVelocity();
    shooterFlywheelAppliedVoltage = shooterFlywheelLeaderTalon.getMotorVoltage();
    shooterFlywheelSupplyCurrent = shooterFlywheelLeaderTalon.getSupplyCurrent();
    shooterFlywheelTorqueCurrent = shooterFlywheelLeaderTalon.getTorqueCurrent();
    shooterFlywheelTempCelsius = shooterFlywheelLeaderTalon.getDeviceTemp();

    closedLoopReferenceSlope = shooterFlywheelLeaderTalon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        shooterFlywheelPosition,
        shooterFlywheelVelocity,
        shooterFlywheelAppliedVoltage,
        shooterFlywheelSupplyCurrent,
        shooterFlywheelTorqueCurrent,
        shooterFlywheelTempCelsius,
        closedLoopReferenceSlope);

    voltageOut = new VoltageOut(0.0);
    positionOut = new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true).withSlot(0);
    motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

    shooterFlywheelLeaderTalon.setPosition(0.0);
  }

  @Override
  public void updateInputs(ShooterFlywheelIOInputs inputs) {
    inputs.shooterFlywheelMotorConnected =
        BaseStatusSignal.refreshAll(
                shooterFlywheelPosition,
                shooterFlywheelVelocity,
                shooterFlywheelAppliedVoltage,
                shooterFlywheelSupplyCurrent,
                shooterFlywheelTorqueCurrent,
                shooterFlywheelTempCelsius,
                closedLoopReferenceSlope)
            .isOK();

    inputs.shooterFlywheelPosition =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(shooterFlywheelPosition, shooterFlywheelVelocity);
    inputs.shooterFlywheelPositionRads = Units.rotationsToRadians(inputs.shooterFlywheelPosition);

    inputs.shooterFlywheelVelocityRadsPerSec = Units.rotationsToRadians(shooterFlywheelVelocity.getValueAsDouble());
    inputs.shooterFlywheelAppliedVoltage = shooterFlywheelAppliedVoltage.getValueAsDouble();
    inputs.shooterFlywheelSupplyCurrentAmps = shooterFlywheelSupplyCurrent.getValueAsDouble();
    inputs.shooterFlywheelTorqueCurrentAmps = shooterFlywheelTorqueCurrent.getValueAsDouble();
    inputs.shooterFlywheelTempCelsius = shooterFlywheelTempCelsius.getValueAsDouble();

    // inputs.motionMagicVelocityTarget =
    //     motorPositionToRads(shooterFlywheelLeaderTalon.getClosedLoopReferenceSlope().getValueAsDouble());
    // inputs.motionMagicPositionTarget =
    //     motorPositionToRads(shooterFlywheelLeaderTalon.getClosedLoopReference().getValueAsDouble());

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    double timeDiff = currentTime - prevReferenceSlopeTimestamp;
    if (timeDiff > 0.0) {
      inputs.acceleration =
          (inputs.motionMagicVelocityTarget - prevClosedLoopReferenceSlope) / timeDiff;
    }
    prevClosedLoopReferenceSlope = inputs.motionMagicVelocityTarget;
    prevReferenceSlopeTimestamp = currentTime;
  }

  @Override
  public void periodicUpdates() {
    updateLoggedTunableNumbers();
  }

  private void updateLoggedTunableNumbers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          slot0Configs.kP = kP.get();
          slot0Configs.kI = kI.get();
          slot0Configs.kD = kD.get();
          slot0Configs.kS = kS.get();
          slot0Configs.kV = kV.get();
          slot0Configs.kA = kA.get();

          StatusCode statusCode = shooterFlywheelLeaderTalon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "ShooterFlywheel Slot 0 Configs",
                    "Error in periodically updating shooterFlywheel Slot0 configs!"));

            Logger.recordOutput("ShooterFlywheel/UpdateSlot0Report", statusCode);
          }
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA);

    LoggedTunableNumber.ifChanged(
        0,
        () -> {
          motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
          motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
          motionMagicConfigs.MotionMagicJerk = motionJerk.get();

          StatusCode statusCode = shooterFlywheelLeaderTalon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "ShooterFlywheel Motion Magic Configs",
                    "Error in periodically updating shooterFlywheel MotionMagic configs!"));

            Logger.recordOutput("ShooterFlywheel/UpdateStatusCodeReport", statusCode);
          }
        },
        motionAcceleration,
        motionCruiseVelocity,
        motionJerk);
  }

  @Override
  public void runVolts(double volts) {
    shooterFlywheelLeaderTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    shooterFlywheelLeaderTalon.stopMotor();
  }

  private double radsToMotorPosition(double rads) {
    return Units.radiansToRotations(rads * reduction);
  }

  private double motorPositionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition / motorToMechanism);
  }
}
