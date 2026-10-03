package org.team5924.frc2026.subsystems.awareness;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import lombok.Getter;

public class FieldZone {
  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(12 * 54 + 2.5975);
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(12 * 26 + 5.6875);

  @Getter private final double blueMinX;
  @Getter private final double blueMaxX;
  @Getter private final double blueMinY;
  @Getter private final double blueMaxY;

  public FieldZone(double blueMinX, double blueMaxX, double blueMinY, double blueMaxY) {
    this.blueMinX = blueMinX;
    this.blueMaxX = blueMaxX;
    this.blueMinY = blueMinY;
    this.blueMaxY = blueMaxY;
  }

  public boolean contains(Pose2d robotPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

    return contains(robotPose, isRed);
  }

  public boolean contains(Pose2d robotPose, boolean isRedAlliance) {
    double x = robotPose.getX();
    double y = robotPose.getY();

    if (isRedAlliance) {
      double redMinX = FIELD_LENGTH_METERS - blueMaxX;
      double redMaxX = FIELD_LENGTH_METERS - blueMinX;

      double redMinY = FIELD_WIDTH_METERS - blueMaxY;
      double redMaxY = FIELD_WIDTH_METERS - blueMinY;

      return x >= redMinX && x <= redMaxX && y >= redMinY && y <= redMaxY;

    } else {
      return x >= blueMinX && x <= blueMaxX && y >= blueMinY && y <= blueMaxY;
    }
  }
}
