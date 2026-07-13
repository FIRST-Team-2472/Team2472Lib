package team2472lib;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.extras.SwerveAutoUtils.directionFromPoseAndTarget;
import static frc.robot.extras.SwerveAutoUtils.getMagnitude;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.extras.SwerveAutoUtils;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveDriveToPointCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance); // Don't automatically flip heading based on
    // alliance

    public PIDController speedPowerController;
    public PIDController turningPowerController;
    private SlewRateLimiter speedDerivLimiter = new SlewRateLimiter(0);
    public SlewRateLimiter turningDerivLimiter = new SlewRateLimiter(0);

    private Rotation2d startingRotation;
    private double progress;
    private double totalDistance;
    private final Pose2d finalPosition;
    private final Timer timer;

    public SwerveDriveToPointCommand(CommandSwerveDrivetrain drivetrain, Pose2d finalPosition) {
        this.drivetrain = drivetrain;
        this.finalPosition = finalPosition; // targetPosition is not field pose
        // but needs mirroring

        timer = new Timer();

        if (RobotBase.isSimulation()) {
            speedPowerController = new PIDController(0.9, 0.0, 0.0);
            turningPowerController = new PIDController(0.9, 0.0, 0.0);
        } else {
            speedPowerController = new PIDController(1.0, 0.0, 0.0);
            turningPowerController = new PIDController(0.6, 0.0, 0.0);
        }

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d botPose = drivetrain.getState().Pose;
        startingRotation = botPose.getRotation();
        progress = 0.0;
        totalDistance = SwerveAutoUtils.getDistance(botPose, finalPosition); // this does get distance
        timer.restart();
    }

    @Override
    public void execute() {
        Pose2d botPose = drivetrain.getState().Pose;

        double currentDistance = SwerveAutoUtils.getDistance(botPose, finalPosition);
        progress = 1 - (currentDistance / totalDistance);
        Rotation2d targetRotation = startingRotation.interpolate(finalPosition.getRotation(), progress);

        double[] direction = directionFromPoseAndTarget(botPose, finalPosition);

        double movementPID = Math.abs(speedPowerController.calculate(0, direction[2]));
        movementPID = Math.min(movementPID, 1.0d);
        double movementSpeed = movementPID * K_AUTO_SPEED;

        // Limit Acceleration
        movementSpeed = speedDerivLimiter.calculate(movementSpeed);

        double turningPID = turningPowerController.calculate(botPose.getRotation().minus(targetRotation).getRadians(), 0);
        turningPID = Math.max(Math.min(turningPID, 1.0d), -1.0);
        double turningSpeed = turningPID * K_MAX_ANGULAR_RATE;

        // Limit Acceleration
        turningSpeed = turningDerivLimiter.calculate(turningSpeed);

        Logger.recordOutput("Movement Speed", movementSpeed);
        Logger.recordOutput("Turning Speed", turningSpeed);

        direction[0] *= movementSpeed;
        direction[1] *= movementSpeed;

        drivetrain.setControl(drive.withVelocityX(direction[0]).withVelocityY(direction[1]).withRotationalRate(turningSpeed));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {

        // use this function if you override the command to finish it
        // if (timer.hasElapsed(20)) {
        // return true;
        // }

        Pose2d botPose = drivetrain.getState().Pose;

        double distance = getMagnitude(botPose.getX() - finalPosition.getX(), botPose.getY() - finalPosition.getY());
        double rotational_error = Math.abs(finalPosition.getRotation().minus(botPose.getRotation()).getDegrees());

        Logger.recordOutput("Movement Distance", distance);
        Logger.recordOutput("Turning Distance", rotational_error);

        boolean isThere = distance < K_AUTO_TRANSLATION_TOLERANCE && rotational_error < K_AUTO_ROTATION_TOLERANCE;

        if (isThere) {
            System.out.printf("%s arrived at position (%.3fm, %.3fm, %.3f°) while being %.3fm and %.3f° off.\n", getName(), botPose.getX(),
                    botPose.getY(), botPose.getRotation().getDegrees(), distance, rotational_error);
        }

        return isThere;
    }
}
