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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.controller.PIDController;


import java.util.function.Consumer;
import java.util.function.Supplier;

public class TeleOpDriveCommand extends Command {
    private final Supplier<Pose2d> pose;
    private final Supplier<ChassisSpeeds> speeds;
    private final Consumer<SwerveRequest> requestConsumer;

    private Supplier<Double> inputX, inputY, inputTheta;
    private Supplier<Boolean> slowMode, intakeActive, aimActive;

    private PIDController yawController;
    private SlewRateLimiter speedLimiter, rotationLimiter;

    private boolean rotationFollowActive, AutoAimActive;
    private boolean isKidMode;
    
    private double maxSpeed, maxRotation;
    private double deadband;
    private double speedMult, rotationMult;
    private double rotationRate, travelRate;

    private double kp, ki, kd;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public final NetworkTable driveStateTable = inst.getTable("DriveState");

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    public TeleOpDriveCommand(
            Subsystem drivetrain, Supplier<Pose2d> pose, 
            Supplier<ChassisSpeeds> speeds,
            Consumer<SwerveRequest> requestConsumer,
            Supplier<Double> inputX, Supplier<Double> inputY, 
            Supplier<Double> inputTheta, Supplier<Boolean> slowMode, 
            Supplier<Boolean> intakeActive, 
            Supplier<Boolean> aimActive) {

        this.pose = pose;
        this.speeds = speeds;

        this.requestConsumer = requestConsumer;

        this.inputX = inputX;
        this.inputY = inputY;
        this.inputTheta = inputTheta;

        this.slowMode = slowMode;
        this.intakeActive = intakeActive;
        this.aimActive = aimActive;

        this.maxSpeed = 6;
        this.maxRotation = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        this.deadband = .1;

        this.speedMult = .3;
        this.rotationMult = .5;

        this.travelRate = 6;
        this.rotationRate = 8;

        this.speedLimiter = new SlewRateLimiter(travelRate);
        this.rotationLimiter = new SlewRateLimiter(rotationRate);

        this.yawController = new PIDController(kp, ki, kd);

        addRequirements(drivetrain);
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

        if (isKidMode) {
            xSpeed /= 10;
            ySpeed /= 10;
            thetaSpeed /= 2;
        }

        xSpeed = speedLimiter.calculate(xSpeed);
        ySpeed = speedLimiter.calculate(ySpeed);
        thetaSpeed = rotationLimiter.calculate(thetaSpeed);

        requestConsumer.accept(drive
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(thetaSpeed)
        );
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

    public TeleOpDriveCommand setRateLimit(double travelRate, double rotationRate) {
        this.travelRate = travelRate;
        this.rotationRate = rotationRate;
        return this;
    }

    public TeleOpDriveCommand setKidMode(boolean isKidMode) {
        this.isKidMode = isKidMode;
        return this;
    }
}
