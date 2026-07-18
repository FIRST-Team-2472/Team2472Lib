package team2472lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
//import frc.robot.MotorPowerController;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotStatus;
import frc.robot.Constants.*;
import frc.robot.extras.AutoAim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class TeleOpDriveCommand extends Command {
    //private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> pose;
    private final Supplier<ChassisSpeeds> speeds;
    private final Consumer<SwerveRequest> requestConsumer;

    private Supplier<Double> inputX, inputY, inputTheta;
    private Supplier<Boolean> slowMode, intakeActive, aimActive;

    private PIDController yawController;
    private SlewRateLimiter xAccelerationLimiter = new SlewRateLimiter(24, -48, 0);
    private SlewRateLimiter yAccelerationLimiter = new SlewRateLimiter(24, -48, 0);
    private SlewRateLimiter thetaAccelerationLimiter = new SlewRateLimiter(180, -225, 0);

    private boolean intakeAutoYawOverrided = false, autoAimYawOverrided = false;
    private boolean isKidMode;
    
    private double maxSpeed = 6;
    private double deadband = .1;
    private double maxRotation = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private double speedMult = .3;
    private double rotationMult = .5;

    private Rotation2d targetAngle = new Rotation2d();
    private AutoAim autoAim = new AutoAim();

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> predictedPose = driveStateTable.getStructTopic("Predicted Pose", Pose2d.struct).publish();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    public TeleOpDriveCommand(
            Subsystem drivetrain, Supplier<Pose2d> pose, 
            Supplier<ChassisSpeeds> speeds,
            Consumer<SwerveRequest> requestConsumer,
            Supplier<Double> inputX, Supplier<Double> inputY, 
            Supplier<Double> inputTheta, Supplier<Boolean> slowMode, 
            Supplier<Boolean> intakeActive, 
            Supplier<Boolean> aimActive, boolean isKidsMode) {
        this.pose = pose;
        this.speeds = speeds;
        this.requestConsumer = requestConsumer;
        this.inputX = inputX;
        this.inputY = inputY;
        this.inputTheta = inputTheta;
        this.slowMode = slowMode;
        this.intakeActive = intakeActive;
        this.aimActive = aimActive;
        this.isKidMode = isKidsMode;
        // TODO: tune pid.
        yawController = new PIDController(0.9, 0.0, 0.0);
        addRequirements(drivetrain);
    }

    public TeleOpDriveCommand(
            Subsystem drivetrain, Supplier<Pose2d> pose, 
            Supplier<ChassisSpeeds> speeds,
            Consumer<SwerveRequest> requestConsumer,
            Supplier<Double> inputX, Supplier<Double> inputY, 
            Supplier<Double> inputTheta, Supplier<Boolean> slowMode, 
            Supplier<Boolean> intakeActive, 
            Supplier<Boolean> aimActive, boolean isKidsMode,
            double maxSpeed, double deadband, double maxRotation, 
            double speedMult, double rotationMult) {
        this(drivetrain, pose, speeds, requestConsumer, 
          inputX, inputY, inputTheta, slowMode, intakeActive,
          aimActive, isKidsMode);
        this.setMaxSpeed(maxSpeed).setDeadband(deadband)
            .setMaxRotation(maxRotation).setSlowModeMult(speedMult, rotationMult);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xSpeed = inputY.get() * (Math.abs(inputY.get()) < deadband ? 0 : 1);
        double ySpeed = inputX.get() * (Math.abs(inputX.get()) < deadband ? 0 : 1);
        double thetaSpeed = inputTheta.get() * (Math.abs(inputTheta.get()) < deadband ? 0 : 1);

        xSpeed *= maxSpeed * (slowMode.get() ? speedMult : 1.0);
        ySpeed *= maxSpeed * (slowMode.get() ? speedMult : 1.0);
        thetaSpeed *= maxRotation * (slowMode.get() ? rotationMult : 1.0);

        // if driver tries to yaw, they disable auto-yaw until they stop intaking and
        // start again
        // TODO: Replace 0.1 with deadband constant
        if (Math.abs(inputTheta.get()) > 0.1 && intakeActive.get()) {
            intakeAutoYawOverrided = true;
        } else if (!intakeActive.get()) {
            intakeAutoYawOverrided = false;
        }

        if ((Math.abs(inputTheta.get()) > 0.1 && aimActive.get()) || isKidMode) {
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

        predictedPose.set(new Pose2d(autoAim.getPredictedPosition(pose.get(), speeds.get()).getX(),
                autoAim.getPredictedPosition(pose.get(), speeds.get()).getY(), targetAngle));

        if (isKidMode) {
            xSpeed /= 10;
            ySpeed /= 10;
            thetaSpeed /= 2;
        }

        requestConsumer.accept(drive
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(thetaSpeed)
        );
        //Logger.recordOutput("Pigeon/Heading", robotYawSignal.refresh().getValueAsDouble());
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted) {}

    public TeleOpDriveCommand setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
        return this;
    }

    public TeleOpDriveCommand setMaxRotation(double maxRotation) {
        this.maxRotation = RotationsPerSecond.of(maxRotation).in(RadiansPerSecond);
        return this;
    }

    public TeleOpDriveCommand setDeadband(double deadband) {
        this.deadband = deadband;
        return this;
    }

    public TeleOpDriveCommand setSlowModeMult(double speedMult, double rotationMult) {
        this.speedMult = speedMult;
        this.rotationMult = rotationMult;
        return this;
    }

    private double getAutoAimYawSpeed() {
        double thetaSpeed;
        // targetAngle = new Rotation2d(autoAim.getTargetYaw(drivetrain.getState().Pose));
        targetAngle = new Rotation2d(autoAim.getTargetYaw(autoAim.getPredictedPosition(pose.get(), speeds.get())));
        targetAngle = targetAngle.minus(Rotation2d.fromRadians(Math.PI));

        Rotation2d angleError = pose.get().getRotation().minus(targetAngle);
        thetaSpeed = yawController.calculate(0, angleError.getRadians());
        return thetaSpeed;
    }

    private double getIntakeAutoYawSpeed(double ySpeed, double xSpeed) {
        double thetaSpeed;
        if (Math.abs(inputX.get()) > deadband || Math.abs(inputY.get()) > deadband) {
            targetAngle = new Rotation2d(Math.atan2(ySpeed, xSpeed));
        } else {
            targetAngle = pose.get().getRotation();
        }

        Rotation2d angleError = pose.get().getRotation().minus(targetAngle);
        thetaSpeed = yawController.calculate(0, angleError.getRadians());
        return thetaSpeed;
    }
}
