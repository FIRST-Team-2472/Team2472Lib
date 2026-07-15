package team2472lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A mutable, chainable 2D vector class designed for fluid math operations.
 * Integrates directly with WPILib geometry types.
 */
public class Vector2D {
    private double x;
    private double y;

    /** Creates a zeroed vector (0, 0). */
    public Vector2D() {
        this(0, 0);
    }

    /** Creates a vector with explicit X and Y components. */
    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /** Creates a vector from a WPILib Translation2d object. */
    public Vector2D(Translation2d translation) {
        this(translation.getX(), translation.getY());
    }

    /** Creates a vector from the positional translation of a WPILib Pose2d. */
    public Vector2D(Pose2d pose) {
        this(pose.getTranslation());
    }

    public static Vector2D fromCoordinates(double x, double y) {
        return new Vector2D(x, y);
    }

    public static Vector2D fromTranslation(Translation2d translation) {
        return new Vector2D(translation);
    }

    public static Vector2D fromPose(Pose2d pose) {
        return new Vector2D(pose);
    }

    public static Vector2D fromPose(Pose2d start, Pose2d end) {
        return new Vector2D(start.relativeTo(end));
    }

    /** Static factory to build a vector using polar coordinates. */
    public static Vector2D fromPolar(double magnitude, Rotation2d angle) {
        return new Vector2D(magnitude * angle.getCos(), magnitude * angle.getSin());
    }

    // --- MUTATING CHAINABLE METHODS (Modifies this instance and returns it) ---

    /** Adds the components of another vector to this instance. */
    public Vector2D add(Vector2D other) {
        this.x += other.x;
        this.y += other.y;
        return this;
    }

    /** Adds a raw translation to this instance. */
    public Vector2D add(Translation2d translation) {
        this.x += translation.getX();
        this.y += translation.getY();
        return this;
    }

    public static Vector2D add(Translation2d a, Translation2d b) {
        return new Vector2D(a.plus(b));
    }

    /** Subtracts the components of another vector from this instance. */
    public Vector2D subtract(Vector2D other) {
        this.x -= other.x;
        this.y -= other.y;
        return this;
    }

    public Vector2D subtract(Translation2d translation) {
        this.x -= translation.getX();
        this.y -= translation.getY();
        return this;
    }

    public static Vector2D subtract(Translation2d a, Translation2d b) {
        return new Vector2D(a.minus(b));
    }

    /** Multiplies the vector components by a constant scalar value. */
    public Vector2D scale(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
        return this;
    }

    public static Vector2D scale(Translation2d vector, double scalar) {
        return new Vector2D(vector.times(scalar));
    }

    /** Rotates the vector counter-clockwise by a given Rotation2d. */
    public Vector2D rotateBy(Rotation2d angle) {
        double cos = angle.getCos();
        double sin = angle.getSin();
        double newX = (this.x * cos) - (this.y * sin);
        double newY = (this.x * sin) + (this.y * cos);
        this.x = newX;
        this.y = newY;
        return this;
    }

    public static Vector2D rotateBy(Translation2d vector, Rotation2d angle) {
        return new Vector2D(vector.rotateBy(angle));
    }

    /** Normalizes the vector (scales its magnitude to 1.0 while protecting its direction). */
    public Vector2D normalize() {
        double mag = getMagnitude();
        if (mag != 0) {
            this.x /= mag;
            this.y /= mag;
        }
        return this;
    }

    public static Vector2D normalize(Translation2d vector) {
        double mag = vector.getNorm();
        if (mag == 0) {
            return new Vector2D();
        }
        return new Vector2D(vector.getX() / mag, vector.getY() / mag);
    }

    // --- TERMINAL / EVALUATING METHODS (Returns raw calculations or WPILib objects) ---

    /** Calculates the scalar magnitude (length) of the vector. */
    public double getMagnitude() {
        return Math.hypot(x, y);
    }

    public static double getMagnitude(double x, double y) {
        return fromCoordinates(x, y).getMagnitude();
    }

    public static double getMagnitude(Translation2d translation) {
        return fromTranslation(translation).getMagnitude();
    }

    public static double getMagnitude(Pose2d pose) {
        return fromPose(pose).getMagnitude();
    }

    public static double getMagnitude(Pose2d start, Pose2d end) {
        return fromPose(start, end).getMagnitude();
    }

    /** Calculates the vector's heading angle. */
    public Rotation2d getAngle() {
        return new Rotation2d(x, y);
    }

    public static Rotation2d getAngle(double x, double y) {
        return fromCoordinates(x, y).getAngle();
    }

    public static Rotation2d getAngle(Translation2d translation) {
        return fromTranslation(translation).getAngle();
    }

    public static Rotation2d getAngle(Pose2d pose) {
        return fromPose(pose).getAngle();
    }

    public static Rotation2d getAngle(Pose2d start, Pose2d end) {
        return fromPose(start, end).getAngle();
    }

    /** Computes the dot product between this vector and another. */
    public double dot(Vector2D other) {
        return (this.x * other.x) + (this.y * other.y);
    }

    public static double dot(double x1, double y1, double x2, double y2) {
        return fromCoordinates(x1, y1).dot(fromCoordinates(x2, y2));
    }

    public static double dot(Translation2d translation1, Translation2d translation2) {
        return fromTranslation(translation1).dot(fromTranslation(translation2));
    }

    public static double dot(Pose2d pose1, Pose2d pose2) {
        return fromPose(pose1).dot(fromPose(pose2));
    }

    public static double dot(Pose2d start1, Pose2d end1, Pose2d start2, Pose2d end2) {
        return fromPose(start1, end1).dot(fromPose(start2, end2));
    }

    /** Computes the 2D cross product magnitude (Z-axis sweep). */
    public double cross(Vector2D other) {
        return (this.x * other.y) - (this.y * other.x);
    }

    public static double cross(double x1, double y1, double x2, double y2) {
        return fromCoordinates(x1, y1).cross(fromCoordinates(x2, y2));
    }

    public static double cross(Translation2d translation1, Translation2d translation2) {
        return fromTranslation(translation1).cross(fromTranslation(translation2));
    }

    public static double cross(Pose2d pose1, Pose2d pose2) {
        return fromPose(pose1).cross(fromPose(pose2));
    }

    public static double cross(Pose2d start1, Pose2d end1, Pose2d start2, Pose2d end2) {
        return fromPose(start1, end1).cross(fromPose(start2, end2));
    }

    /** Converts this object back into a clean WPILib Translation2d instance. */
    public Translation2d toTranslation2d() {
        return new Translation2d(x, y);
    }

    /** Converts this object into a WPILib Pose2d instance pointing along the vector's angle. */
    public Pose2d toPose2d() {
        return new Pose2d(toTranslation2d(), getAngle());
    }

    public double[] toArray() {
        double mag = this.getMagnitude();
        double[] array = {x, y, mag};
        return array;
    }

    public double getX() { return x; }
    public double getY() { return y; }

    @Override
    public String toString() {
        return String.format("Vector2D(X: %.3f, Y: %.3f, Mag: %.3f)", x, y, getMagnitude());
    }
}