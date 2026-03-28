/*
 * HopperElevatorIOTalonFX.java
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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class HopperElevatorIOTalonFX implements HopperElevatorIO {
  private final TalonFX talon;
  private final CANcoder cancoder;

  /* Configurators */
  private final TalonFXConfigurator talonConfig;

  /* Configs  */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpointMeters;
  /* Logged Tunable Numbers */
  private final LoggedTunableNumber kP = new LoggedTunableNumber("HopperElevator/kP", 40.0); //TODO: tune all these
  private final LoggedTunableNumber kI = new LoggedTunableNumber("HopperElevator/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("HopperElevator/kD", 0.0);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("HopperElevator/kS", 0.2);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("HopperElevator/kV", 0.0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("HopperElevator/kG", 2.8);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("HopperElevator/kA", 0.0);

  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("HopperElevator/MotionCruiseVelocity", 100.0);
  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("HopperElevator/MotionAcceleration", 1000.0);
  private final LoggedTunableNumber motionJerk =
      new LoggedTunableNumber("HopperElevator/MotionJerk", 0.0);
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final StatusSignal<Angle> cancoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> cancoderVelocity;
  private final StatusSignal<Voltage> cancoderSupplyVoltage;
  private final StatusSignal<Angle> cancoderPositionRotations;

  private final StatusSignal<Double> closedLoopReferenceSlope;
  private double prevClosedLoopReferenceSlope = 0.0;
  private double prevReferenceSlopeTimestamp = 0.0;

  private final VoltageOut voltageOut;
  private final MotionMagicTorqueCurrentFOC motionMagicCurrent;

  public HopperElevatorIOTalonFX() {
    talon = new TalonFX(Constants.HopperElevator.CAN_ID, new CANBus(Constants.HopperElevator.BUS));
    cancoder =
        new CANcoder(
            Constants.HopperElevator.CANCODER_ID, new CANBus(Constants.HopperElevator.BUS));
    talonConfig = talon.getConfigurator();

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();
    slot0Configs.kG = kG.get();
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[7];

    statusArray[0] = talonConfig.apply(Constants.HopperElevator.CONFIG);
    statusArray[1] = talonConfig.apply(slot0Configs);
    statusArray[2] = talonConfig.apply(motionMagicConfigs);
    statusArray[3] = talonConfig.apply(Constants.GENERIC_OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[4] = talonConfig.apply(Constants.GENERIC_CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[5] = talonConfig.apply(Constants.HopperElevator.FEEDBACK_CONFIGS);
    statusArray[6] = cancoder.getConfigurator().apply(Constants.HopperElevator.CANCODER_CONFIGS);
    boolean isErrorPresent = false;
    for (StatusCode status : statusArray) if (!status.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              "Shooter Hood Configs",
              "Error in applying Shooter Hood configs!"));

    Logger.recordOutput("ShooterHood/InitConfReport", statusArray);

    // Get select status signals and set update frequency
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    cancoderAbsolutePosition = cancoder.getAbsolutePosition();
    cancoderVelocity = cancoder.getVelocity();
    cancoderSupplyVoltage = cancoder.getSupplyVoltage();
    cancoderPositionRotations = cancoder.getPosition();

    closedLoopReferenceSlope = talon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        cancoderAbsolutePosition,
        cancoderVelocity,
        cancoderSupplyVoltage,
        cancoderPositionRotations,
        closedLoopReferenceSlope);

    voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    motionMagicCurrent = new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(100.0).withSlot(0);

    BaseStatusSignal.waitForAll(0.5, cancoderAbsolutePosition);

    // assume elevator is fully stowed at robot turn on
    cancoder.setPosition(0.0);
    talon.setPosition(0.0);
  }

  @Override
  public void updateInputs(HopperElevatorIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                closedLoopReferenceSlope)
            .isOK();

    inputs.position = BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
    inputs.positionRads = position.getValueAsDouble();
    inputs.velocityRadsPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();

    inputs.positionMeters = getHeight();
    inputs.velMetersPerSecond = getVelocity();

    inputs.motionMagicVelocityTarget =
        rotationsToMeters(talon.getClosedLoopReferenceSlope().getValue());
    inputs.motionMagicPositionTarget = rotationsToMeters(talon.getClosedLoopReference().getValue());

    inputs.setpointMeters = setpointMeters;

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    double timeDiff = currentTime - prevReferenceSlopeTimestamp;
    if (timeDiff > 0.0) {
      inputs.acceleration =
          (inputs.motionMagicVelocityTarget - prevClosedLoopReferenceSlope) / timeDiff;
    }
    prevClosedLoopReferenceSlope = inputs.motionMagicVelocityTarget;
    prevReferenceSlopeTimestamp = currentTime;
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
          slot0Configs.kG = kG.get();
          slot0Configs.kA = kA.get();

          StatusCode statusCode = talon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "IntakePivot Slot 0 Configs",
                    "Error in periodically updating intakePivot Slot0 configs!"));

            Logger.recordOutput("IntakePivot/UpdateSlot0Report", statusCode);
          }
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kG,
        kA);

    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        () -> {
          motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
          motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
          motionMagicConfigs.MotionMagicJerk = motionJerk.get();

          StatusCode statusCode = talon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "IntakePivot Motion Magic Configs",
                    "Error in periodically updating intakePivot MotionMagic configs!"));

            Logger.recordOutput("IntakePivot/UpdateStatusCodeReport", statusCode);
          }
        },
        motionAcceleration,
        motionCruiseVelocity,
        motionJerk);
  }

  @Override
  public void periodicUpdates() {
    updateLoggedTunableNumbers();
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setHeight(double heightMeters) {
    setpointMeters = heightMeters;
    talon.setControl(motionMagicCurrent.withPosition(metersToRotations(heightMeters)));
  }

  private double metersToRotations(double height) { // Double check the math here
    return height
        * Constants.HopperElevator.MOTOR_TO_MECHANISM
        / (2 * Math.PI * Constants.HopperElevator.PULLEY_RADIUS_METERS);
  }

  public double rotationsToMeters(double rotations) {
    return (rotations
        * 2
        * Math.PI
        * Constants.HopperElevator.PULLEY_RADIUS_METERS
        / Constants.HopperElevator.MOTOR_TO_MECHANISM);
  }

  public void stop() {
    talon.stopMotor();
  }

  private double getHeight() {
    return rotationsToMeters(talon.getPosition().getValueAsDouble());
  }

  private double getVelocity() {
    return rotationsToMeters(talon.getVelocity().getValueAsDouble());
  }
}
