/*
 * DriveSimulation.java
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

package org.team5924.frc2026.subsystems.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.generated.TunerConstants;

/**
 * Owns the physics simulation of the drivetrain and the simulated field. This is the only place
 * that talks to the maple-sim arena, so the physics engine can be swapped out without touching the
 * rest of the robot code. Only construct this in SIM mode.
 */
public class DriveSimulation {
  /** Where the simulated robot starts, and where it returns when the field is reset. */
  public static final Pose2d STARTING_POSE = new Pose2d(3, 3, new Rotation2d());

  private static final DriveTrainSimulationConfig CONFIG =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(Drive.ROBOT_MASS_KG))
          .withCustomModuleTranslations(Drive.getModuleTranslations())
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getFalcon500(1),
                  TunerConstants.FrontLeft.DriveMotorGearRatio,
                  TunerConstants.FrontLeft.SteerMotorGearRatio,
                  Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                  Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                  Meters.of(TunerConstants.FrontLeft.WheelRadius),
                  KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                  Drive.WHEEL_COF));

  private final SwerveDriveSimulation swerveDriveSimulation;

  /** Creates the drivetrain simulation at {@link #STARTING_POSE} and adds it to the arena. */
  public DriveSimulation() {
    swerveDriveSimulation = new SwerveDriveSimulation(CONFIG, STARTING_POSE);
    SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
  }

  /** The robot's true pose on the simulated field, independent of odometry. */
  public Pose2d getGroundTruthPose() {
    return swerveDriveSimulation.getSimulatedDriveTrainPose();
  }

  /** Teleports the simulated robot to the given pose. */
  public void resetPose(Pose2d pose) {
    swerveDriveSimulation.setSimulationWorldPose(pose);
  }

  /** Resets the field's game pieces to their auto starting positions. */
  public void resetFieldForAuto() {
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  /** Steps the physics simulation. Call once per robot loop. */
  public void periodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", getGroundTruthPose());
  }

  public GyroSimulation getGyroSimulation() {
    return swerveDriveSimulation.getGyroSimulation();
  }

  /** Module simulations, ordered FL, FR, BL, BR. */
  public SwerveModuleSimulation getModuleSimulation(int index) {
    return swerveDriveSimulation.getModules()[index];
  }

  /** Timestamps of the odometry samples the simulation produced during the last robot loop. */
  public static double[] getOdometryTimestamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    final double periodSeconds =
        SimulatedArena.getSimulationSubTicksIn1Period()
            * SimulatedArena.getSimulationDt().in(Seconds);
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp()
              - periodSeconds
              + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }
}
