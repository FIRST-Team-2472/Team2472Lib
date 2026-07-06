package team2472lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Use for points along a path for autos, as `AutoPose2d` can automatically flip for the red alliance. Please do not use outside of that purpose, use
 * a `Pose2d` for internal math
 */
public class AutoPose2d extends Pose2d {
    public double fieldLength;
    public double fieldWidth;
    public AutoPose2d(double fieldLength, double fieldWidth) {
        this.fieldLength = fieldLength;
        this.fieldWidth = fieldWidth;
    }

    public AutoPose2d(Pose2d pose2d, double fieldLength, double fieldWidth) {
        super(pose2d.getTranslation(), pose2d.getRotation());
        this.fieldLength = fieldLength;
        this.fieldWidth = fieldWidth;
    }

    public AutoPose2d(double x, double y, Rotation2d angle, double fieldLength, double fieldWidth) {
        super(x, y, angle);
        this.fieldLength = fieldLength;
        this.fieldWidth = fieldWidth;
    }

    public Pose2d toPose2d() {
        DriverStation.Alliance alliance;

        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        return alliance == DriverStation.Alliance.Red ? new Pose2d(fieldLength - getX(), fieldWidth - getY(), getRotation())
                : new Pose2d(getX(), getY(), getRotation());
    }
}
