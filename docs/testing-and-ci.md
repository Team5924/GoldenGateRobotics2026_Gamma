# Testing and CI

This explains how we test robot code and how CI checks every pull request. The last section is the plan for setting up the same thing in the 2027 (Systemcore) repo.

## What CI checks

Every pull request runs **Build and test** (`.github/workflows/build.yml`) inside WPILib's `roborio-cross-ubuntu:2025-24.04` container:

1. `./gradlew spotlessCheck`: fails if any file isn't formatted. Fix it with `./gradlew spotlessApply`. Local builds format automatically; CI only checks.
2. `./gradlew build`: compiles and runs every test in `src/test/java`.

When a run fails, open it on GitHub, scroll to the failed step, and download the **test-reports** artifact for the full HTML report. `main` requires this check to pass before merging.

## Running tests locally

```bash
./gradlew test
```

Results are in `build/reports/tests/test/index.html` and coverage is in `build/reports/jacoco/test/html/index.html`. In VS Code, the Testing panel can run a single test.

## How our tests are built

Tests run on your laptop with no robot. They use three tools, from cheapest to most involved.

### 1. Pure-logic tests

For math and helpers that don't touch hardware (`Bounds`, `ElevatorUtil`, `AllianceFlipUtil`, `LaunchCalculator`). Plain JUnit, and `@ParameterizedTest` for tables of cases. See `src/test/java/org/team5924/frc2026/util/`.

### 2. Subsystem tests with fake IO

Every subsystem takes its IO in the constructor (AdvantageKit's IO pattern), so a test can hand it a **fake** instead of a TalonFX:

```java
FlywheelIOFake io = new FlywheelIOFake();
Flywheel flywheel = new Flywheel(io);

flywheel.setGoalState(FlywheelState.FAST_LAUNCH);
io.velocityRotationsPerSec = 30.0; // pretend the wheel is up to speed
flywheel.periodic();

assertEquals(FlywheelIOFake.Command.VELOCITY, io.lastCommand);
assertTrue(RobotState.getInstance().isFlywheelReady());
```

Each fake (`*IOFake.java`, next to the subsystem's tests) records the **last command** the subsystem sent and reports whatever **sensor values** the test sets. To test a new subsystem, write a fake for its IO interface the same way.

### 3. `RobotTestBase`

Extend `testutil/RobotTestBase` in any test that uses WPILib, the command scheduler, `RobotState` or `MatchState`. Before each test it:

- starts the HAL
- resets the simulated driver station (disabled, no alliance)
- pauses time
- empties the command scheduler
- creates fresh `RobotState` and `MatchState` singletons

Helpers:

- `setAlliance(Alliance.Red)` and `setDriverStation(alliance, enabled, autonomous)`
- `stepTime(seconds)`, which moves simulated time forward

### Rules

- **Never sleep.** Time is paused; use `stepTime(...)`.
- **No shared state between tests.** If you add a new singleton, give it a `resetForTests()` method and call it from `RobotTestBase`.
- **Don't boot `Robot` or `RobotContainer`** in unit tests. Build only the subsystem you're testing, with fake IO.
- **Compare angles by difference** (`a.minus(b).getRadians()`), not by `getDegrees()`. Wrapping differs between years.
- **A test that exposes a bug** goes in as `@Disabled("Bug: ... <issue link>")`. The fix PR removes `@Disabled`. Run the disabled tests with `src/test/resources/junit-platform.properties` containing `junit.jupiter.conditions.deactivate=org.junit.*DisabledCondition` (don't commit that file).

### Not tested here yet

We don't yet have tests for:

- **Swerve drive, vision and autos.** These depend on maple-sim, CTRE sim timing and Driver Station APIs that all change in 2027, so headless auto tests wait for the 2027 repo.
- **`LaunchCalculator.getParameters()`.** It's a singleton with filters, and it needs a reset hook first.

## Starting the 2027 (Systemcore) repo

WPILib 2027 drops roboRIO support, so 2027 code lives in a new repo rather than a branch of this one. When setting it up:

1. Create it from the WPILib 2027 AdvantageKit **skeleton** template (the only 2027 template).
2. Copy over `build.gradle`'s Spotless, Jacoco and `test {}` settings.
3. Copy `src/test/java/.../testutil/RobotTestBase.java`, the `*IOFake` classes, and any tests for code that carries over. Most IO-fake tests only need import and constant renames.
4. Add this workflow as `.github/workflows/build.yml`. WPILib hasn't published 2027 CI guidance, so there is no container yet; the runner plus a Java 25 JDK is enough for Java.

   ```yaml
   name: Build
   on:
     push:
       branches: [main]
     pull_request:
   concurrency:
     group: ${{ github.workflow }}-${{ github.ref }}
     cancel-in-progress: true
   jobs:
     build:
       name: Build and test
       runs-on: ubuntu-26.04 # WPILib 2027 supports only the latest Ubuntu LTS
       steps:
         - uses: actions/checkout@v4
         - uses: actions/setup-java@v4
           with:
             distribution: temurin
             java-version: '25' # WPILib 2027 uses Java 25
             cache: gradle
         - run: chmod +x gradlew
         - run: ./gradlew spotlessCheck
         - run: ./gradlew build
         - uses: actions/upload-artifact@v4
           if: always()
           with:
             name: test-reports
             path: |
               build/reports/
               build/test-results/
             if-no-files-found: ignore
   ```

5. Confirm a deliberately failing test turns the PR red, then require **Build and test** on `main`.

### Things in *this* codebase that will break in 2027

All 2027 software is alpha; check the [WPILib 2027 changelog](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html) and the [SystemcoreTesting repo](https://github.com/wpilibsuite/SystemcoreTesting) before each upgrade.

- **Name clash:** WPILib 2027 adds its own `MatchState` class. Rename ours (`org.team5924.frc2026.MatchState`), for example to `GameMatchState`.
- **Driver Station:** about 30 `DriverStation.*` calls across 16 files move to `MatchState`/`RobotState`. In tests, `DriverStationSim` setup moves to the `RobotMode` enum. Only `RobotTestBase.setDriverStation` needs to change.
- **Microseconds become nanoseconds** for integer timestamps. Audit:
  - `MatchState.getTime()`: `Logger.getTimestamp() * 1.0E-6`
  - `PhoenixOdometryThread`: `RobotController.getFPGATime() / 1e6`
  - `PhoenixUtil` and `ModuleIOSim`: `Timer.getFPGATimestamp()`. This one returns seconds, but re-check it.
- **Dashboards:** `SmartDashboard.putData` in `Drive` moves to the Telemetry API. Both `LoggedDashboardChooser`s in `RobotContainer` become `LoggedNetworkChooser`.
- **Angles:** `Rotation2d.getDegrees()` now wraps. Re-check the comparison in `DriveCommands.joystickDriveWhileLaunching`.
- **CTRE:** devices take a `CANBus` built from a `CANPort` instead of a bus name or int. There is no REV or Limelight code to port.
- **maple-sim** has no 2027 build. Drive sim falls back to per-module `DCMotorSim`; the unused `ModuleIOSim` here is a starting point. Re-add maple-sim when it's ported.
- **Constants:** WPILib constants become ALL_CAPS (`Rotation2d.kZero` → `Rotation2d.ZERO`, `kPi` → `PI`), and packages move from `edu.wpi.first` to `org.wpilib`. The VS Code importer handles most of this.
