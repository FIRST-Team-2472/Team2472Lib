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

    public Vector2D(Pose2d start, Pose2d end) {
        this(start.relativeTo(end));
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

    /** Subtracts the components of another vector from this instance. */
    public Vector2D subtract(Vector2D other) {
        this.x -= other.x;
        this.y -= other.y;
        return this;
    }

    /** Multiplies the vector components by a constant scalar value. */
    public Vector2D scale(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
        return this;
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

    /** Normalizes the vector (scales its magnitude to 1.0 while protecting its direction). */
    public Vector2D normalize() {
        double mag = getMagnitude();
        if (mag != 0) {
            this.x /= mag;
            this.y /= mag;
        }
        return this;
    }

    // --- TERMINAL / EVALUATING METHODS (Returns raw calculations or WPILib objects) ---

    /** Calculates the scalar magnitude (length) of the vector. */
    public double getMagnitude() {
        return Math.hypot(x, y);
    }

    /** Calculates the vector's heading angle. */
    public Rotation2d getAngle() {
        return new Rotation2d(x, y);
    }

    /** Computes the dot product between this vector and another. */
    public double dot(Vector2D other) {
        return (this.x * other.x) + (this.y * other.y);
    }

    /** Computes the 2D cross product magnitude (Z-axis sweep). */
    public double cross(Vector2D other) {
        return (this.x * other.y) - (this.y * other.x);
    }

    /** Converts this object back into a clean WPILib Translation2d instance. */
    public Translation2d toTranslation2d() {
        return new Translation2d(x, y);
    }

    /** Converts this object into a WPILib Pose2d instance pointing along the vector's angle. */
    public Pose2d toPose2d() {
        return new Pose2d(toTranslation2d(), getAngle());
    }

    public double getX() { return x; }
    public double getY() { return y; }

    @Override
    public String toString() {
        return String.format("Vector2D(X: %.3f, Y: %.3f, Mag: %.3f)", x, y, getMagnitude());
    }
}