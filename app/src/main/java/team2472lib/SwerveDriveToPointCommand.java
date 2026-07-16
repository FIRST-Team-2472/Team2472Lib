package team2472lib;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;


public class SwerveDriveToPointCommand extends Command {

    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<SwerveRequest> requestConsumer;

    private final Pose2d targetPose;
    private final double maxSpeed;
    private final double maxRotation;
    
    private double distanceTolerance = 0.01;
    private double rotationTolerance = .2;

    private final PIDController translationPID;
    private final PIDController rotationPID;
    private final SlewRateLimiter speedLimiter;
    private final SlewRateLimiter rotationLimiter;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);

    private Rotation2d startingRotation;
    private double totalDistance;

    private Timer timer;

    public SwerveDriveToPointCommand(
            Subsystem drivetrain,
            Supplier<Pose2d> poseSupplier,
            Consumer<SwerveRequest> requestConsumer,
            Pose2d targetPose,
            double maxSpeed,
            double maxRotation) {

        this.poseSupplier = poseSupplier;
        this.requestConsumer = requestConsumer;
        this.targetPose = targetPose;
        this.maxSpeed = maxSpeed;
        this.maxRotation = maxRotation;

        timer = new Timer();

        if (RobotBase.isSimulation()) {
            translationPID = new PIDController(0.9, 0.0, 0.0);
            rotationPID = new PIDController(0.9, 0.0, 0.0);
        } else {
            translationPID = new PIDController(1.0, 0.0, 0.0);
            rotationPID = new PIDController(0.6, 0.0, 0.0);
        }

        this.speedLimiter = new SlewRateLimiter(6.0);
        this.rotationLimiter = new SlewRateLimiter(8.0);

        addRequirements(drivetrain);
    }

    public SwerveDriveToPointCommand(
            Subsystem drivetrain,
            Supplier<Pose2d> poseSupplier,
            Consumer<SwerveRequest> requestConsumer,
            Pose2d targetPose,
            double maxSpeed,
            double maxRotation, 
            double distanceTolerance, 
            double rotationTolerance) {

        this(drivetrain, poseSupplier, requestConsumer, targetPose, maxSpeed, maxRotation);
        setTolerance(distanceTolerance, rotationTolerance);
    }

    public void setTolerance(double distanceTolerance, double rotationTolerance) {
        this.distanceTolerance = distanceTolerance;
        this.rotationTolerance = rotationTolerance;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = poseSupplier.get();
        startingRotation = currentPose.getRotation();
    
        totalDistance = Vector2D.fromPose(currentPose, targetPose).getMagnitude();
        
        timer.restart();
    }

    @Override
    public void execute() {
        Pose2d currentPose = poseSupplier.get();

        double currentDistance = Vector2D.fromPose(currentPose, targetPose).getMagnitude();
        Vector2D direction = Vector2D.fromPose(currentPose, targetPose).normalize();

        double progress = totalDistance > 0 ? 1.0 - (currentDistance / totalDistance) : 1.0;
        progress = Math.max(0.0, Math.min(1.0, progress)); // Clamp progress between 0 and 1
        Rotation2d targetHeading = startingRotation.interpolate(targetPose.getRotation(), progress);

        double translationOutput = translationPID.calculate(currentDistance, 0);
        translationOutput = Math.min(Math.max(translationOutput, -1.0), 1.0); // Clamp to [-1, 1]
        double requestedSpeed = translationOutput * maxSpeed;
        double limitedSpeed = speedLimiter.calculate(requestedSpeed);

        double rotationOutput = rotationPID.calculate(targetHeading.minus(currentPose.getRotation()).getRadians(), 0);
        rotationOutput = Math.min(Math.max(rotationOutput, -1.0), 1.0); // Clamp to [-1, 1]
        double requestedRotation = rotationOutput * maxRotation;
        double limitedRotation = rotationLimiter.calculate(requestedRotation);

        requestConsumer.accept(driveRequest
                .withVelocityX(direction.getX() * limitedSpeed)
                .withVelocityY(direction.getY() * limitedSpeed)
                .withRotationalRate(limitedRotation)
        );
    }

    @Override
    public void end(boolean interrupted) {
        requestConsumer.accept(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = poseSupplier.get();

        double distance = Vector2D.getMagnitude(currentPose, targetPose);
        double rotationError = Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());
        
        boolean isThere = (distance < distanceTolerance) && (rotationError < rotationTolerance);

        if (isThere) {
            System.out.printf("[SwerveDriveToPoint] Arrived! Offsets: %.3fm, %.1f°\n", 
                    distance, rotationError);
        }

        return isThere;
    }
}
