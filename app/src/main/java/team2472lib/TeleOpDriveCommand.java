package team2472lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

//import frc.robot.MotorPowerController;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotStatus;
import frc.robot.Constants.*;
import frc.robot.extras.AutoAim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class TeleOpDriveCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private Supplier<Double> inputX, inputY, inputTheta;
    private Supplier<Boolean> slowMode, intakeActive, aimActive;
    private MotorPowerController yawController;
    private boolean intakeAutoYawOverrided = false, autoAimYawOverrided = false;
    private SlewRateLimiter xAccelerationLimiter = new SlewRateLimiter(24, -48, 0);
    private SlewRateLimiter yAccelerationLimiter = new SlewRateLimiter(24, -48, 0);
    private SlewRateLimiter thetaAccelerationLimiter = new SlewRateLimiter(180, -225, 0);

    private Rotation2d targetAngle = new Rotation2d();
    private AutoAim autoAim = new AutoAim();

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    public final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> predictedPose = driveStateTable.getStructTopic("Predicted Pose", Pose2d.struct).publish();
    private final StatusSignal<Angle> robotYawSignal;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    public TeleOpDriveCommand(CommandSwerveDrivetrain drivetrain, Supplier<Double> inputX, Supplier<Double> inputY, Supplier<Double> inputTheta,
            Supplier<Boolean> slowMode, Supplier<Boolean> intakeActive, Supplier<Boolean> aimActive) {
        this.drivetrain = drivetrain;
        this.inputX = inputX;
        this.inputY = inputY;
        this.inputTheta = inputTheta;
        this.slowMode = slowMode;
        this.intakeActive = intakeActive;
        this.aimActive = aimActive;
        // TODO: tune pid.
        yawController = new MotorPowerController(DriveConstants.K_YAW_CONTROLLER_KP, DriveConstants.K_YAW_CONTROLLER_KI,
                DriveConstants.K_YAW_CONTROLLER_D_TIME, DriveConstants.K_YAW_CONTROLLER_TIME, DriveConstants.K_YAW_CONTROLLER_ALLOWED_ERROR,
                DriveConstants.K_YAW_CONTROLLER_INITIAL_SENSOR_READ, DriveConstants.K_YAW_CONTROLLER_INTEGRAL_PROPORTIONAL_THRESHOLD);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        robotYawSignal = drivetrain.getPigeon2().getYaw();
        addRequirements(this.drivetrain);
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is called repeatedly until {@link #isFinished()}
     * returns true.)
     */
    @Override
    public void execute() {
        double xSpeed = inputY.get() * (Math.abs(inputY.get()) < DriveConstants.K_DEADBAND ? 0 : 1);
        double ySpeed = inputX.get() * (Math.abs(inputX.get()) < DriveConstants.K_DEADBAND ? 0 : 1);
        double thetaSpeed = inputTheta.get() * (Math.abs(inputTheta.get()) < DriveConstants.K_DEADBAND ? 0 : 1);

        xSpeed *= DriveConstants.K_MAX_SPEED * (slowMode.get() ? DriveConstants.K_SLOW_MODE_MULTIPLIER : 1.0);
        ySpeed *= DriveConstants.K_MAX_SPEED * (slowMode.get() ? DriveConstants.K_SLOW_MODE_MULTIPLIER : 1.0);
        thetaSpeed *= DriveConstants.K_MAX_ANGULAR_RATE * (slowMode.get() ? DriveConstants.K_SLOW_MODE_ROTATIONAL_MULTIPLIER : 1.0);

        // if driver tries to yaw, they disable auto-yaw until they stop intaking and
        // start again
        // TODO: Replace 0.1 with deadband constant
        if (Math.abs(inputTheta.get()) > 0.1 && intakeActive.get()) {
            intakeAutoYawOverrided = true;
        } else if (!intakeActive.get()) {
            intakeAutoYawOverrided = false;
        }

        if ((Math.abs(inputTheta.get()) > 0.1 && aimActive.get()) || RobotStatus.isKidMode) {
            autoAimYawOverrided = true;
        } else if (!aimActive.get()) {
            autoAimYawOverrided = false;
        }

        // if using intake and not using manual rotation control
        if (intakeActive.get() && !intakeAutoYawOverrided) {

            thetaSpeed = getIntakeAutoYawSpeed(ySpeed, xSpeed);
        } else if (aimActive.get() && !autoAimYawOverrided) {// may need to tweak how this yaw controller is used
            // if auto aim and not using manual rotation control

            thetaSpeed = getAutoAimYawSpeed();
        }

        predictedPose.set(new Pose2d(autoAim.getPredictedPosition(drivetrain.getState().Pose, drivetrain.getState().Speeds).getX(),
                autoAim.getPredictedPosition(drivetrain.getState().Pose, drivetrain.getState().Speeds).getY(), targetAngle));

        if (RobotStatus.isKidMode) {
            xSpeed /= 10;
            ySpeed /= 10;
            thetaSpeed /= 2;
        }

        drivetrain.setControl(drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(thetaSpeed));

        Logger.recordOutput("Pigeon/Heading", robotYawSignal.refresh().getValueAsDouble());
    }

    private double getAutoAimYawSpeed() {
        double thetaSpeed;
        // targetAngle = new Rotation2d(autoAim.getTargetYaw(drivetrain.getState().Pose));
        targetAngle = new Rotation2d(autoAim.getTargetYaw(autoAim.getPredictedPosition(drivetrain.getState().Pose, drivetrain.getState().Speeds)));
        targetAngle = targetAngle.minus(Rotation2d.fromRadians(Math.PI));

        Rotation2d angleError = drivetrain.getState().Pose.getRotation().minus(targetAngle);
        thetaSpeed = yawController.calculate(0, angleError.getRadians());
        return thetaSpeed;
    }

    private double getIntakeAutoYawSpeed(double ySpeed, double xSpeed) {
        double thetaSpeed;
        if (Math.abs(inputX.get()) > DriveConstants.K_DEADBAND || Math.abs(inputY.get()) > DriveConstants.K_DEADBAND) {
            targetAngle = new Rotation2d(Math.atan2(ySpeed, xSpeed));
        } else {
            targetAngle = drivetrain.getState().Pose.getRotation();
        }

        Rotation2d angleError = drivetrain.getState().Pose.getRotation().minus(targetAngle);
        thetaSpeed = yawController.calculate(0, angleError.getRadians());
        return thetaSpeed;
    }

    /**
     * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true -- the scheduler will call its
     * {@link #end(boolean)} method.
     *
     * <p>
     * Returning false will result in the command never ending automatically. It may still be cancelled manually or interrupted by another command.
     * Hard coding this command to always return true will result in the command executing once and finishing immediately. It is recommended to use *
     * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally -- that is, it is called when {@link #isFinished()}
     * returns true -- or when it is interrupted/canceled. This is where you may want to wrap up loose ends, like shutting off a motor that was being
     * used in the command.
     *
     * @param interrupted
     *            whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
    }
}
