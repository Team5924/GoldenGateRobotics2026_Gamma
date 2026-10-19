# GoldenGateRobotics2026_Gamma

[![Build](https://github.com/Team5924/GoldenGateRobotics2026_Gamma/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/Team5924/GoldenGateRobotics2026_Gamma/actions/workflows/build.yml)

Robot code for Team 5924 Golden Gate Robotics' 2026 robot.

## Common commands

| Command | What it does |
| --- | --- |
| `./gradlew build` | Compile, format, and run all tests (what CI runs) |
| `./gradlew test` | Run the unit tests only |
| `./gradlew spotlessApply` | Format the code (also runs automatically on every local compile) |
| `./gradlew spotlessCheck` | Check formatting without changing files (CI runs this) |
| `./gradlew simulateJava` | Run the robot in desktop simulation; view it in AdvantageScope |
| `./gradlew deploy` | Deploy to the robot |

Test results are in `build/reports/tests/test/index.html` and coverage is in `build/reports/jacoco/test/html/index.html`.

See [docs/testing-and-ci.md](docs/testing-and-ci.md) for how to write tests and for the 2027 (Systemcore) CI plan.
