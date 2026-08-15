/*
 * RobotContainer.java
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

package org.team5924.frc2026;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team5924.frc2026.Constants.Mode;
import org.team5924.frc2026.commands.AutoBuilder;
import org.team5924.frc2026.commands.drive.DriveCommands;
import org.team5924.frc2026.commands.shooter.AutoScoreCommands;
import org.team5924.frc2026.generated.TunerConstants;
import org.team5924.frc2026.subsystems.drive.Drive;
import org.team5924.frc2026.subsystems.drive.GyroIO;
import org.team5924.frc2026.subsystems.drive.GyroIOPigeon2;
import org.team5924.frc2026.subsystems.drive.GyroIOSim;
import org.team5924.frc2026.subsystems.drive.ModuleIO;
import org.team5924.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.team5924.frc2026.subsystems.drive.ModuleIOTalonFXSim;
import org.team5924.frc2026.subsystems.flywheel.Flywheel;
import org.team5924.frc2026.subsystems.flywheel.Flywheel.FlywheelState;
import org.team5924.frc2026.subsystems.flywheel.FlywheelIO;
import org.team5924.frc2026.subsystems.flywheel.FlywheelIOSim;
import org.team5924.frc2026.subsystems.flywheel.FlywheelIOTalonFX;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot.IntakePivotState;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivotIO;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivotIOSim;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivotIOTalonFX;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHood;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHood.ShooterHoodState;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHoodIO;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHoodIOSim;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHoodIOTalonFX;
import org.team5924.frc2026.subsystems.rollers.hopper.Hopper;
import org.team5924.frc2026.subsystems.rollers.hopper.Hopper.HopperState;
import org.team5924.frc2026.subsystems.rollers.hopper.HopperIO;
import org.team5924.frc2026.subsystems.rollers.hopper.HopperIOSim;
import org.team5924.frc2026.subsystems.rollers.hopper.HopperIOTalonFX;
import org.team5924.frc2026.subsystems.rollers.indexer.Indexer;
import org.team5924.frc2026.subsystems.rollers.indexer.IndexerIO;
import org.team5924.frc2026.subsystems.rollers.indexer.IndexerIOSim;
import org.team5924.frc2026.subsystems.rollers.indexer.IndexerIOTalonFX;
import org.team5924.frc2026.subsystems.rollers.intake.Intake;
import org.team5924.frc2026.subsystems.rollers.intake.Intake.IntakeState;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIO;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIOSim;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIOTalonFX;
import org.team5924.frc2026.subsystems.vision.Vision;
import org.team5924.frc2026.subsystems.vision.VisionConstants;
import org.team5924.frc2026.subsystems.vision.VisionIOPhotonVision;
import org.team5924.frc2026.subsystems.vision.VisionIOPhotonVisionSim;
import org.team5924.frc2026.util.LaunchCalculator;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private SwerveDriveSimulation driveSimulation = null;
  private final Vision vision;
  private final Intake intake;
  private final IntakePivot intakePivot;
  private final Hopper hopper;
  private final Indexer indexer;
  private final ShooterHood shooterHood;
  private final Flywheel flywheel;

  // Real/IO implementation
  private final boolean realDrive = true;
  private final boolean realVision = true;

  private final boolean realIntake = true;
  private final boolean realIntakePivot = true;

  private final boolean realHopper = true;
  private final boolean realIndexer = true;

  private final boolean realShooterHood = true;
  private final boolean realFlywheel = true;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoFactory autoFactory;

  private enum ControlMode {
    NONE,
    TUNING,
    NORMAL
  }

  private Drive buildDriveSystem() {
    if (Constants.currentMode == Mode.REPLAY
        || (Constants.currentMode == Mode.REAL && !realDrive)) {
      return new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          (pose) -> {});
    }

    if (Constants.currentMode == Mode.REAL) {
      return new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight),
          (pose) -> {});
    }

    // === SIM ===
    driveSimulation =
        new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

    return new Drive(
        new GyroIOSim(driveSimulation.getGyroSimulation()),
        new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
        new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
        new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
        new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
        driveSimulation::setSimulationWorldPose);
  }

  private Vision buildVisionSystem() {
    if (Constants.currentMode == Mode.REPLAY
        || (Constants.currentMode == Mode.REAL && !realVision)) {
      return null;
    }

    // TODO: add left and right cameras once we receive measurements

    if (Constants.currentMode == Mode.REAL) {
      VisionIOPhotonVision frontCamera =
          new VisionIOPhotonVision(VisionConstants.FRONT_NAME, VisionConstants.FRONT_TRANSFORM);
      // VisionIOPhotonVision leftCamera =
      //   new VisionIOPhotonVision(VisionConstants.LEFT_NAME, VisionConstants.LEFT_TRANSFORM);
      // VisionIOPhotonVision rightCamera =
      //   new VisionIOPhotonVision(VisionConstants.RIGHT_NAME, VisionConstants.RIGHT_TRANSFORM);

      return new Vision(drive::addVisionMeasurement, frontCamera);
    }

    // === SIM ===
    VisionIOPhotonVisionSim frontCameraSim =
        new VisionIOPhotonVisionSim(
            VisionConstants.FRONT_NAME,
            VisionConstants.FRONT_TRANSFORM,
            VisionConstants.SIM_THRIFTYCAM_PROPERTIES,
            driveSimulation::getSimulatedDriveTrainPose);

    // VisionIOPhotonVisionSim leftCameraSim =
    //   new VisionIOPhotonVisionSim(
    //     VisionConstants.LEFT_NAME,
    //     VisionConstants.LEFT_TRANSFORM,
    //     VisionConstants.SIM_THRIFTYCAM_PROPERTIES,
    //     driveSimulation::getSimulatedDriveTrainPose);

    // VisionIOPhotonVisionSim rightCameraSim =
    //   new VisionIOPhotonVisionSim(
    //     VisionConstants.RIGHT_NAME,
    //     VisionConstants.RIGHT_TRANSFORM,
    //     VisionConstants.SIM_THRIFTYCAM_PROPERTIES,
    //     driveSimulation::getSimulatedDriveTrainPose);

    return new Vision(drive::addVisionMeasurement, frontCameraSim);
  }

  private Intake buildIntakeSystem() {
    if (Constants.currentMode == Mode.REPLAY || (Constants.currentMode == Mode.REAL && !realIntake))
      return new Intake(new IntakeIO() {});

    if (Constants.currentMode == Mode.REAL) return new Intake(new IntakeIOTalonFX());

    return new Intake(new IntakeIOSim());
  }

  private IntakePivot buildIntakePivotSystem() {
    if (Constants.currentMode == Mode.REPLAY
        || (Constants.currentMode == Mode.REAL && !realIntakePivot))
      return new IntakePivot(new IntakePivotIO() {});

    if (Constants.currentMode == Mode.REAL) return new IntakePivot(new IntakePivotIOTalonFX());

    return new IntakePivot(new IntakePivotIOSim());
  }

  private Hopper buildHopperSystem() {
    if (Constants.currentMode == Mode.REPLAY || (Constants.currentMode == Mode.REAL && !realHopper))
      return new Hopper(new HopperIO() {});

    if (Constants.currentMode == Mode.REAL) return new Hopper(new HopperIOTalonFX());

    return new Hopper(new HopperIOSim());
  }

  private Indexer buildIndexerSystem() {
    if (Constants.currentMode == Mode.REPLAY
        || (Constants.currentMode == Mode.REAL && !realIndexer))
      return new Indexer(new IndexerIO() {});

    if (Constants.currentMode == Mode.REAL) return new Indexer(new IndexerIOTalonFX());

    return new Indexer(new IndexerIOSim());
  }

  private ShooterHood buildShooterHoodSystem() {
    if (Constants.currentMode == Mode.REPLAY
        || (Constants.currentMode == Mode.REAL && !realShooterHood))
      return new ShooterHood(new ShooterHoodIO() {});

    if (Constants.currentMode == Mode.REAL) return new ShooterHood(new ShooterHoodIOTalonFX());

    return new ShooterHood(new ShooterHoodIOSim());
  }

  private Flywheel buildFlywheelSystem() {
    if (Constants.currentMode == Mode.REPLAY
        || (Constants.currentMode == Mode.REAL && !realFlywheel))
      return new Flywheel(new FlywheelIO() {});

    if (Constants.currentMode == Mode.REAL) return new Flywheel(new FlywheelIOTalonFX());

    return new Flywheel(new FlywheelIOSim());
  }

  private void registerAutoCommands() {
    NamedCommands.registerCommand(
        "Run Shooter",
        Commands.run(() -> AutoScoreCommands.runTrackTargetCommand(shooterHood, flywheel))
            .finallyDo(() -> flywheel.setGoalState(FlywheelState.IDLE)));

    NamedCommands.registerCommand(
        "Run Intake",
        Commands.runOnce(() -> intake.setGoalState(IntakeState.INTAKE))
            .finallyDo(() -> intake.setGoalState(IntakeState.INTAKE)));
  }

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    drive = buildDriveSystem();
    vision = buildVisionSystem();

    intake = buildIntakeSystem();
    intakePivot = buildIntakePivotSystem();

    hopper = buildHopperSystem();
    indexer = buildIndexerSystem();

    shooterHood = buildShooterHoodSystem();
    flywheel = buildFlywheelSystem();

    autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followChoreoTrajectory, true, drive);

    registerAutoCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    LoggedDashboardChooser<String> startingPosition =
        new LoggedDashboardChooser<>("Starting Position?");
    startingPosition.addOption("Middle", "Mid");
    startingPosition.addDefaultOption("Right", "Right");
    startingPosition.addOption("Left", "Left");
    AutoBuilder.setStartingPosition(startingPosition::get);
    var autoBuilder =
        new AutoBuilder(
            autoFactory, drive, shooterHood, flywheel, intake, intakePivot, hopper, indexer);

    // autoChooser.addOption("Score and Climb Auto", autoBuilder.scoreAndClimbAuto());
    // autoChooser.addOption("Score, Depot, and Climb Auto", autoBuilder.scorePickupAndClimbAuto());
    autoChooser.addDefaultOption("Right Double Swipe", autoBuilder.rightDoubleSwipe());
    autoChooser.addOption("Left Double Swipe", autoBuilder.leftDoubleSwipe());
    autoChooser.addOption("Left Double Swipe Intake", autoBuilder.leftDoubleSwipeIntake());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // config the button bindings
    configButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configButtonBindings() {
    configDriveBindings();

    configDefaultCommands();

    configureIntakeBindings(false, ControlMode.NORMAL);
    configureShooterBindings(false, ControlMode.NORMAL);
  }

  private void configDriveBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    // [driver a] -> SLOW MODE YIPE
    driveController
        .x()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY() * Constants.SLOW_MODE_MULTI,
                () -> -driveController.getLeftX() * Constants.SLOW_MODE_MULTI,
                () -> -driveController.getRightX() * Constants.SLOW_MODE_MULTI));

    // [driver y] -> auto launch
    driveController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveWhileLaunching(
                drive, () -> -driveController.getLeftY(), () -> -driveController.getLeftX()));

    // // [driver] Switch to X pattern when X button is pressed
    // driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    final Runnable resetGyro =
        (Constants.currentMode == Constants.Mode.SIM)
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    final Runnable resetGyroInverted =
        (Constants.currentMode == Constants.Mode.SIM)
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.k180deg));

    driveController
        .start()
        .onTrue(Commands.runOnce(resetGyroInverted, drive).ignoringDisable(true));

    // [a] -> Reset gyro to 0°
    driveController.a().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  private void configDefaultCommands() {
    // hopper on by default
    hopper.setDefaultCommand(
        Commands.run(() -> hopper.setGoalState(Hopper.HopperState.ON), hopper));

    // flywheel on by default
    flywheel.setDefaultCommand(
        Commands.runOnce(() -> flywheel.setGoalState(FlywheelState.IDLE), flywheel));
  }

  // intake + intake pivot
  public void configureIntakeBindings(boolean manual, ControlMode controlMode) {
    if (manual) {
      // [operator right y] -> manual intake pivot
      intakePivot.setDefaultCommand(
          Commands.run(
              () -> intakePivot.runManual(() -> -operatorController.getRightY()), intakePivot));
    }

    switch (controlMode) {
      case TUNING -> {
        // [driver right bumper] -> turn off intake pivot
        driveController
            .rightBumper()
            .onTrue(
                Commands.runOnce(
                    () -> intakePivot.setGoalState(IntakePivotState.OFF), intakePivot));

        // [driver left bumper] -> stow intake pivot
        driveController
            .leftBumper()
            .onTrue(
                Commands.runOnce(
                    () -> intakePivot.setGoalState(IntakePivotState.STOW), intakePivot));

        // [driver right trigger] -> center intake pivot
        driveController
            .rightTrigger()
            .onTrue(
                Commands.runOnce(
                    () -> intakePivot.setGoalState(IntakePivotState.CENTER), intakePivot));

        // [driver left trigger] -> deploy intake pivot
        driveController
            .leftTrigger()
            .onTrue(
                Commands.runOnce(
                    () -> intakePivot.setGoalState(IntakePivotState.DOWN), intakePivot));
      }
      case NORMAL -> {
        // [driver right bumper] -> deploy intake pivot, run intake
        driveController
            .rightBumper()
            .onTrue(
                Commands.runOnce(
                    () -> {
                      intakePivot.setGoalState(IntakePivotState.DOWN);
                      intake.setGoalState(IntakeState.INTAKE);
                    },
                    intakePivot,
                    intake));

        // [driver left bumper] -> intake pivot shooting mode, run intake
        driveController
            .leftBumper()
            .onTrue(
                Commands.runOnce(
                    () -> {
                      intakePivot.setGoalState(IntakePivotState.SHOOTING_UP);
                      intake.setGoalState(IntakeState.INTAKE);
                    },
                    intakePivot,
                    intake));

        // [dpad down] -> stow intake pivot, stop intake = panic intake button
        driveController
            .povDown()
            .onTrue(
                Commands.runOnce(
                    () -> {
                      intakePivot.setGoalState(IntakePivotState.STOW);
                      intake.setGoalState(IntakeState.OFF);
                    },
                    intakePivot,
                    intake));
      }
      case NONE -> {}
    }
  }

  private void configureShooterBindings(boolean manual, ControlMode controlMode) {
    if (manual) {
      shooterHood.setDefaultCommand(
          Commands.run(
              () -> shooterHood.runManual(() -> -operatorController.getRightY()), shooterHood));
    }

    switch (controlMode) {
      case TUNING -> {
        operatorController
            .rightBumper()
            .onTrue(
                Commands.runOnce(
                    () -> shooterHood.setGoalState(ShooterHoodState.OFF), shooterHood));

        operatorController
            .leftBumper()
            .onTrue(
                Commands.runOnce(
                    () -> shooterHood.setGoalState(ShooterHoodState.BOTTOM), shooterHood));

        operatorController
            .rightTrigger()
            .onTrue(
                Commands.runOnce(
                    () -> shooterHood.setGoalState(ShooterHoodState.CENTER), shooterHood));

        operatorController
            .leftTrigger()
            .onTrue(
                Commands.runOnce(
                    () -> shooterHood.setGoalState(ShooterHoodState.MAX), shooterHood));
      }
      case NORMAL -> {
        // [right trigger pressed] -> shoot
        driveController
            .rightTrigger()
            .and(() -> LaunchCalculator.getInstance().getParameters().isValid())
            .onTrue(
                Commands.parallel(
                    Commands.runOnce(
                        () -> {
                          shooterHood.setGoalState(ShooterHoodState.AUTO);
                          indexer.setGoalState(Indexer.IndexerState.INDEXING);
                        },
                        shooterHood,
                        indexer),
                    Commands.run(
                        () -> flywheel.setGoalState(Flywheel.FlywheelState.AUTO), flywheel)));

        // [right trigger released] -> stop shoot
        driveController
            .rightTrigger()
            .onFalse(
                Commands.runOnce(
                    () -> {
                      shooterHood.setGoalState(ShooterHoodState.OFF);
                      flywheel.setGoalState(Flywheel.FlywheelState.IDLE);
                      indexer.setGoalState(Indexer.IndexerState.OFF);
                    },
                    shooterHood,
                    flywheel,
                    indexer));

        // [dpad up] -> panic shoot (when auto shoot doesn't work)
        driveController
            .povUp()
            .onTrue(
                Commands.parallel(
                    Commands.runOnce(
                        () -> {
                          shooterHood.setGoalState(ShooterHoodState.CENTER);
                          indexer.setGoalState(Indexer.IndexerState.INDEXING);
                          hopper.setGoalState(HopperState.ON);
                        },
                        shooterHood,
                        indexer,
                        hopper),
                    Commands.run(
                        () -> flywheel.setGoalState(Flywheel.FlywheelState.SLOW_LAUNCH),
                        flywheel)));

        // [dpad up release] -> stop panic shoot
        driveController
            .povUp()
            .onFalse(
                Commands.runOnce(
                    () -> {
                      shooterHood.setGoalState(ShooterHoodState.OFF);
                      flywheel.setGoalState(Flywheel.FlywheelState.IDLE);
                      indexer.setGoalState(Indexer.IndexerState.OFF);
                    },
                    shooterHood,
                    flywheel,
                    indexer));
      }
      case NONE -> {}
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
