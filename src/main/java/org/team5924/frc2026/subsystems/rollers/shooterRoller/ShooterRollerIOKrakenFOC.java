/*
 * ShooterRollerIOTalonFX.java
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

package org.team5924.frc2026.subsystems.rollers.shooterRoller;

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

public class ShooterRollerIOKrakenFOC implements ShooterRollerIO {
  /* Hardware */
  private final TalonFX shooterRollerLeaderTalon;
  private final TalonFX shooterRollerFollowerTalon;

  /* Configurators */
  private TalonFXConfigurator shooterRollerTalonLeaderConfig;
  private TalonFXConfigurator shooterRollerTalonFollowerConfig;

  /* Configs  */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpointRads;

  /* Gains */
  private final LoggedTunableNumber kP = new LoggedTunableNumber("ShooterRoller/kP", 3.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("ShooterRoller/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("ShooterRoller/kD", 0.07);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("ShooterRoller/kS", 0.13);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("ShooterRoller/kV", 0.4);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("ShooterRoller/kA", 0.00);

  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("ShooterRoller/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("ShooterRoller/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerk = 
      new LoggedTunableNumber("ShooterRoller/MotionJerk", 0.0);

  /* Status Signals */
  private final StatusSignal<Angle> shooterRollerPosition;
  private final StatusSignal<AngularVelocity> shooterRollerVelocity;
  private final StatusSignal<Voltage> shooterRollerAppliedVoltage;
  private final StatusSignal<Current> shooterRollerSupplyCurrent;
  private final StatusSignal<Current> shooterRollerTorqueCurrent;
  private final StatusSignal<Temperature> shooterRollerTempCelsius;


  private final StatusSignal<Double> closedLoopReferenceSlope;
  private double prevClosedLoopReferenceSlope = 0.0;
  private double prevReferenceSlopeTimestamp = 0.0;

  private final VoltageOut voltageOut;
  private final PositionVoltage positionOut;
  private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;

  public ShooterRollerIOKrakenFOC(boolean isLeft) {
    shooterRollerLeaderTalon =
        new TalonFX(
            isLeft ? Constants.ShooterRollerLeaderLeft.CAN_ID : Constants.ShooterRollerLeaderRight.CAN_ID,
            new CANBus(isLeft ? Constants.ShooterRollerLeaderLeft.BUS : Constants.ShooterRollerLeaderRight.BUS));

    shooterRollerFollowerTalon =
        new TalonFX(
            isLeft ? Constants.ShooterRollerFollowerLeft.CAN_ID : Constants.ShooterRollerFollowerRight.CAN_ID,
            new CANBus(isLeft ? Constants.ShooterRollerFollowerLeft.BUS : Constants.ShooterRollerFollowerRight.BUS));

    shooterRollerTalonLeaderConfig = shooterRollerLeaderTalon.getConfigurator();
    shooterRollerTalonFollowerConfig = shooterRollerFollowerTalon.getConfigurator();

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
        shooterRollerTalonLeaderConfig.apply(
            isLeft ? Constants.ShooterRollerLeaderLeft.CONFIG : Constants.ShooterRollerLeaderRight.CONFIG);
    statusArray[1] =
        shooterRollerTalonLeaderConfig.apply(
            isLeft
                ? Constants.ShooterRollerLeaderLeft.OPEN_LOOP_RAMPS_CONFIGS
                : Constants.ShooterRollerLeaderRight.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[2] =
        shooterRollerTalonLeaderConfig.apply(
            isLeft
                ? Constants.ShooterRollerLeaderLeft.CLOSED_LOOP_RAMPS_CONFIGS
                : Constants.ShooterRollerLeaderRight.CLOSED_LOOP_RAMPS_CONFIGS);

    statusArray[3] =
        shooterRollerTalonLeaderConfig.apply(
            isLeft ? Constants.ShooterRollerFollowerLeft.CONFIG : Constants.ShooterRollerFollowerRight.CONFIG);
    statusArray[4] =
        shooterRollerTalonLeaderConfig.apply(
            isLeft
                ? Constants.ShooterRollerFollowerLeft.OPEN_LOOP_RAMPS_CONFIGS
                : Constants.ShooterRollerFollowerRight.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[5] =
        shooterRollerTalonLeaderConfig.apply(
            isLeft
                ? Constants.ShooterRollerFollowerLeft.CLOSED_LOOP_RAMPS_CONFIGS
                : Constants.ShooterRollerFollowerRight.CLOSED_LOOP_RAMPS_CONFIGS);

    statusArray[6] = shooterRollerTalonLeaderConfig.apply(motionMagicConfigs);
    statusArray[7] = shooterRollerTalonLeaderConfig.apply(slot0Configs);

    statusArray[6] = shooterRollerTalonLeaderConfig.apply(motionMagicConfigs);
    statusArray[7] = shooterRollerTalonLeaderConfig.apply(slot0Configs);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING, "ShooterRoller Configs", "Error in shooterRoller configs!"));

    Logger.recordOutput("ShooterRoller/InitConfReport", statusArray);

    // Get select status signals and set update frequency
    shooterRollerPosition = shooterRollerLeaderTalon.getPosition();
    shooterRollerVelocity = shooterRollerLeaderTalon.getVelocity();
    shooterRollerAppliedVoltage = shooterRollerLeaderTalon.getMotorVoltage();
    shooterRollerSupplyCurrent = shooterRollerLeaderTalon.getSupplyCurrent();
    shooterRollerTorqueCurrent = shooterRollerLeaderTalon.getTorqueCurrent();
    shooterRollerTempCelsius = shooterRollerLeaderTalon.getDeviceTemp();

    closedLoopReferenceSlope = shooterRollerLeaderTalon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        shooterRollerPosition,
        shooterRollerVelocity,
        shooterRollerAppliedVoltage,
        shooterRollerSupplyCurrent,
        shooterRollerTorqueCurrent,
        shooterRollerTempCelsius,
        closedLoopReferenceSlope);

    voltageOut = new VoltageOut(0.0);
    positionOut = new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true).withSlot(0);
    motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

    shooterRollerLeaderTalon.setPosition(0.0);
  }

  @Override
  public void updateInputs(ShooterRollerIOInputs inputs) {
    inputs.shooterRollerMotorConnected =
        BaseStatusSignal.refreshAll(
                shooterRollerPosition,
                shooterRollerVelocity,
                shooterRollerAppliedVoltage,
                shooterRollerSupplyCurrent,
                shooterRollerTorqueCurrent,
                shooterRollerTempCelsius,
                closedLoopReferenceSlope)
            .isOK();

    inputs.shooterRollerPosition =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(shooterRollerPosition, shooterRollerVelocity);
    inputs.shooterRollerPositionRads = Units.rotationsToRadians(inputs.shooterRollerPosition);

    inputs.shooterRollerVelocityRadsPerSec = Units.rotationsToRadians(shooterRollerVelocity.getValueAsDouble());
    inputs.shooterRollerAppliedVoltage = shooterRollerAppliedVoltage.getValueAsDouble();
    inputs.shooterRollerSupplyCurrentAmps = shooterRollerSupplyCurrent.getValueAsDouble();
    inputs.shooterRollerTorqueCurrentAmps = shooterRollerTorqueCurrent.getValueAsDouble();
    inputs.shooterRollerTempCelsius = shooterRollerTempCelsius.getValueAsDouble();

    // inputs.motionMagicVelocityTarget =
    //     motorPositionToRads(shooterRollerLeaderTalon.getClosedLoopReferenceSlope().getValueAsDouble());
    // inputs.motionMagicPositionTarget =
    //     motorPositionToRads(shooterRollerLeaderTalon.getClosedLoopReference().getValueAsDouble());

    inputs.setpointRads = setpointRads;

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

          StatusCode statusCode = shooterRollerLeaderTalon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "ShooterRoller Slot 0 Configs",
                    "Error in periodically updating shooterRoller Slot0 configs!"));

            Logger.recordOutput("ShooterRoller/UpdateSlot0Report", statusCode);
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

          StatusCode statusCode = shooterRollerLeaderTalon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "ShooterRoller Motion Magic Configs",
                    "Error in periodically updating shooterRoller MotionMagic configs!"));

            Logger.recordOutput("ShooterRoller/UpdateStatusCodeReport", statusCode);
          }
        },
        motionAcceleration,
        motionCruiseVelocity,
        motionJerk);
  }

  @Override
  public void runVolts(double volts) {
    shooterRollerLeaderTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    shooterRollerLeaderTalon.stopMotor();
  }
}
