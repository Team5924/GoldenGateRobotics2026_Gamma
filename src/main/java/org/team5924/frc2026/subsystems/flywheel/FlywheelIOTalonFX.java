/*
 * FlywheelIOTalonFX.java
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

package org.team5924.frc2026.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.Constants.Flywheel;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class FlywheelIOTalonFX implements FlywheelIO {
  /* Hardware */
  private final TalonFX leftTopTalon;
  private final TalonFX leftBottomTalon;
  private final TalonFX rightTopTalon;
  private final TalonFX rightBottomTalon;

  /* Configurators */
  private final TalonFXConfigurator leftTopConfig;
  private final TalonFXConfigurator leftBottomConfig;
  private final TalonFXConfigurator rightTopConfig;
  private final TalonFXConfigurator rightBottomConfig;

  /* Configs  */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpointVelocityRotationsPerSec;

  /* Gains  */
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 999999999.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheel/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA", 4.788);

  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("Flywheel/MotionAcceleration", 400.0);
  private final LoggedTunableNumber motionJerk =
      new LoggedTunableNumber("Flywheel/MotionJerk", 4000.0);

  /* Status Signals */
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final ArrayList<StatusSignal<Temperature>> tempCelsius = new ArrayList<>(4);

  private final StatusSignal<Double> closedLoopReferenceSlope;
  private double prevClosedLoopReferenceSlope = 0.0;
  private double prevReferenceSlopeTimestamp = 0.0;

  private final VoltageOut voltageOut;
  private final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocity;

  public FlywheelIOTalonFX() {
    leftTopTalon = new TalonFX(Flywheel.LEFT_TOP_ID, new CANBus(Flywheel.BUS));
    leftBottomTalon = new TalonFX(Flywheel.LEFT_BOTTOM_ID, new CANBus(Flywheel.BUS));
    rightTopTalon = new TalonFX(Flywheel.RIGHT_TOP_ID, new CANBus(Flywheel.BUS));
    rightBottomTalon = new TalonFX(Flywheel.RIGHT_BOTTOM_ID, new CANBus(Flywheel.BUS));

    leftTopConfig = leftTopTalon.getConfigurator();
    leftBottomConfig = leftBottomTalon.getConfigurator();
    rightTopConfig = rightTopTalon.getConfigurator();
    rightBottomConfig = rightBottomTalon.getConfigurator();

    slot0Configs = new Slot0Configs();
    updateSlot0Configs();

    motionMagicConfigs = new MotionMagicConfigs();
    updateMotionMagicConfigs();

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[10];

    statusArray[0] = leftTopConfig.apply(Flywheel.CONFIG);
    statusArray[1] = leftTopConfig.apply(Constants.GENERIC_OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[2] = leftTopConfig.apply(Constants.GENERIC_CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[3] = leftTopConfig.apply(Flywheel.FEEDBACK_CONFIGS);
    statusArray[4] = leftTopConfig.apply(Flywheel.TORQUE_CURRENT_CONFIGS);
    statusArray[5] = leftTopConfig.apply(slot0Configs);
    statusArray[6] = leftTopConfig.apply(motionMagicConfigs);

    statusArray[7] = leftBottomConfig.apply(Flywheel.CONFIG);
    statusArray[8] = rightTopConfig.apply(Flywheel.CONFIG);
    statusArray[9] = rightBottomConfig.apply(Flywheel.CONFIG);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING, "Flywheel Configs", "Error in shooter flywheel configs!"));

    Logger.recordOutput("Flywheel/InitConfReport", statusArray);

    leftBottomTalon.setControl(new Follower(Flywheel.LEFT_TOP_ID, MotorAlignmentValue.Aligned));
    rightTopTalon.setControl(new Follower(Flywheel.LEFT_TOP_ID, MotorAlignmentValue.Opposed));
    rightBottomTalon.setControl(new Follower(Flywheel.LEFT_TOP_ID, MotorAlignmentValue.Opposed));

    // Get select status signals and set update frequency
    position = leftTopTalon.getPosition();
    velocity = leftTopTalon.getVelocity();
    appliedVoltage = leftTopTalon.getMotorVoltage();
    supplyCurrent = leftTopTalon.getSupplyCurrent();
    torqueCurrent = leftTopTalon.getTorqueCurrent();

    tempCelsius.add(leftTopTalon.getDeviceTemp());
    tempCelsius.add(leftBottomTalon.getDeviceTemp());
    tempCelsius.add(rightTopTalon.getDeviceTemp());
    tempCelsius.add(rightBottomTalon.getDeviceTemp());

    closedLoopReferenceSlope = leftTopTalon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius.get(0),
        tempCelsius.get(1),
        tempCelsius.get(2),
        tempCelsius.get(3));

    ParentDevice.optimizeBusUtilizationForAll(
        leftTopTalon, leftBottomTalon, rightBottomTalon, rightTopTalon);

    voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    motionMagicVelocity = new MotionMagicVelocityTorqueCurrentFOC(0.0).withSlot(0);

    leftTopTalon.setPosition(0.0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.motorConnected[0] =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius.get(0),
                closedLoopReferenceSlope)
            .isOK();

    for (int i = 1; i < 4; ++i) {
      inputs.motorConnected[i] = BaseStatusSignal.refreshAll(tempCelsius.get(i)).isOK();
    }

    inputs.position = BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
    inputs.positionRads = Units.rotationsToRadians(inputs.position);

    inputs.velocityRotationsPerSec = velocity.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();

    for (int i = 0; i < 4; ++i) {
      inputs.tempCelsius[i] = tempCelsius.get(i).getValueAsDouble();
    }

    inputs.motionMagicVelocityTarget =
        motorPositionToRads(leftTopTalon.getClosedLoopReferenceSlope().getValueAsDouble());

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    double timeDiff = currentTime - prevReferenceSlopeTimestamp;
    if (timeDiff > 0.0) {
      inputs.acceleration =
          (inputs.motionMagicVelocityTarget - prevClosedLoopReferenceSlope) / timeDiff;
    }
    prevClosedLoopReferenceSlope = inputs.motionMagicVelocityTarget;
    prevReferenceSlopeTimestamp = currentTime;

    inputs.setpointVelocityRotationsPerSec = setpointVelocityRotationsPerSec;
  }

  @Override
  public void periodicUpdates() {
    updateLoggedTunableNumbers();
  }

  private void updateSlot0Configs() {
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();
  }

  private void updateMotionMagicConfigs() {
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();
  }

  private void updateLoggedTunableNumbers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          updateSlot0Configs();

          StatusCode statusCode = leftTopConfig.apply(slot0Configs);
          if (!statusCode.isOK()) {
            Logger.recordOutput("Flywheel/UpdateSlot0Report", statusCode);
          }
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA);

    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        () -> {
          updateMotionMagicConfigs();

          StatusCode statusCode = leftTopConfig.apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Logger.recordOutput("Flywheel/UpdateStatusCodeReport", statusCode);
          }
        },
        motionAcceleration,
        motionJerk);
  }

  @Override
  // runs motor at volts
  public void runVolts(double volts) {
    leftTopTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setVelocity(double velocityRotationsPerSec) {
    setpointVelocityRotationsPerSec = velocityRotationsPerSec;
    leftTopTalon.setControl(motionMagicVelocity.withVelocity(setpointVelocityRotationsPerSec));
  }

  @Override
  public void stop() {
    setpointVelocityRotationsPerSec = 0.0;
    leftTopTalon.stopMotor();
  }

  private double radsToMotorPosition(double rads) {
    return Units.radiansToRotations(rads);
  }

  private double motorPositionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition);
  }
}
