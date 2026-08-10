// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team2472lib.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

public class Limelight extends SubsystemBase {
    
    /** Helper data container to send vision updates back to the drivetrain */
    public static record VisionMeasurement(Pose2d pose, double timestamp, Vector<N3> stdDevs) {}
    

    private final String llName;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Double> yawVelocitySupplier;
    private final Consumer<VisionMeasurement> visionConsumer;
    private final Supplier<Boolean> disableButton;
    private final Predicate<Pose2d> bumpChecker;

    private boolean enable = false;
    private boolean trust = false;
    private boolean hitBumpRecently = false;
    private int fieldError = 0;
    private int distanceError = 0;
    private Pose2d llPose;
    
    private final boolean isMegaTag2;
    private final boolean isLL4;
    private boolean wasDisabled = false;
    private double heartbeat = 0;
    private final Timer timeSinceBump = new Timer();

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    /**
     * Main Decoupled Limelight Subsystem.
     * * @@param name Name of the Limelight in NetworkTables (e.g., "limelight")
     * @param isMegaTag2 Whether to use MegaTag2 pose estimation algorithms
     * @param isLL4 Set true if using Limelight 4 hardware
     * @param poseSupplier Supplier providing the robot's current Pose2d from odometry
     * @param yawVelocitySupplier Supplier providing angular yaw velocity in degrees/sec
     * @param visionConsumer Consumer callback to pass valid vision measurements back to the drivetrain
     * @param disableButton Supplier checking if vision updates should be manually disabled
     * @param bumpChecker Optional predicate to check if the robot is currently on a field bump/obstacle
     */
    public Limelight(
            String name,
            boolean isMegaTag2,
            boolean isLL4,
            Supplier<Pose2d> poseSupplier,
            Supplier<Double> yawVelocitySupplier,
            Consumer<VisionMeasurement> visionConsumer,
            Supplier<Boolean> disableButton,
            Predicate<Pose2d> bumpChecker) {

        this.llName = name;
        this.isMegaTag2 = isMegaTag2;
        this.isLL4 = isLL4;
        this.poseSupplier = poseSupplier;
        this.yawVelocitySupplier = yawVelocitySupplier;
        this.visionConsumer = visionConsumer;
        this.disableButton = disableButton != null ? disableButton : () -> false;
        this.bumpChecker = bumpChecker != null ? bumpChecker : (pose) -> false;

        SmartDashboard.putNumber(llName + "/Field Error", fieldError);
        SmartDashboard.putNumber(llName + "/Limelight Error", distanceError);

        LimelightHelpers.SetIMUAssistAlpha(llName, 0.01);
        timeSinceBump.restart();
    }

    /** Overloaded simple constructor without bump checking */
    public Limelight(
            String name,
            boolean isMegaTag2,
            boolean isLL4,
            Supplier<Pose2d> poseSupplier,
            Supplier<Double> yawVelocitySupplier,
            Consumer<VisionMeasurement> visionConsumer,
            Supplier<Boolean> disableButton) {
        this(name, isMegaTag2, isLL4, poseSupplier, yawVelocitySupplier, visionConsumer, disableButton, null);
    }

    @Override
    public void periodic() {
        if (disableButton.get()) {
            wasDisabled = true;
        }

        boolean isDisabled = DriverStation.isDisabled();
        
        // 1. Pipeline and IMU states
        if (isDisabled) {
            LimelightHelpers.setPipelineIndex(llName, 1); // Low-power / idle pipeline
        } else {
            LimelightHelpers.setPipelineIndex(llName, 0); // Active tracking pipeline
        }
        LimelightHelpers.SetIMUMode(llName, 1); // Mode 1: Seed internal IMU with Pigeon

        // 2. Supply continuous yaw/pose data for MegaTag2 / IMU algorithms
        Pose2d robotPose = poseSupplier.get();
        double currentYawVelocity = yawVelocitySupplier.get();

        LimelightHelpers.SetRobotOrientation(llName, robotPose.getRotation().getDegrees(), currentYawVelocity, 0.0, 0.0, 0.0, 0.0);

        double heartbeatNow = LimelightHelpers.getHeartbeat(llName);
        if (heartbeat >= heartbeatNow) {
            return; // Limelight offline or disconnected
        }
        
        if (bumpChecker.test(robotPose)) {
            timeSinceBump.restart();
        }
        hitBumpRecently = !timeSinceBump.hasElapsed(5);

        heartbeat = heartbeatNow;

        if (enable && !isSpinningTooFast(currentYawVelocity)) {
            LimelightHelpers.PoseEstimate poseEstimate;

            if (isMegaTag2) {
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            } else {
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);
            }

            //if (poseEstimate == null) return;

            double targetDistance = poseEstimate.avgTagDist;
            double confidence = Math.max(((targetDistance - 1) / 6), 0.01);

            double[] rawStdDevs = NetworkTableInstance.getDefault().getTable(llName).getEntry("stddevs").getDoubleArray(new double[12]);

            Vector<N3> stdDevs = VecBuilder.fill(confidence, confidence, Double.MAX_VALUE);
            
            if (rawStdDevs != null && rawStdDevs.length >= 8) {
                if (isMegaTag2) {
                    if (hitBumpRecently && !wasDisabled) {
                        stdDevs = VecBuilder.fill(rawStdDevs[6], rawStdDevs[7], Double.MAX_VALUE);
                        SmartDashboard.putBoolean(llName + "/hit bump recently", true);
                    } else {
                        stdDevs = VecBuilder.fill(rawStdDevs[6] * 2.5, rawStdDevs[7] * 2.5, Double.MAX_VALUE);
                        SmartDashboard.putBoolean(llName + "/hit bump recently", false);
                    }
                } else {
                    stdDevs = VecBuilder.fill(rawStdDevs[0] * 2.5, rawStdDevs[1] * 2.5, Double.MAX_VALUE);
                }
            }

            if (poseEstimate.tagCount > 0) {
                this.llPose = new Pose2d(poseEstimate.pose.getTranslation(), robotPose.getRotation());

                if (isPoseWithinFieldBoundaries(this.llPose)) {
                    if (robotPose.getTranslation().getDistance(this.llPose.getTranslation()) < 0.5 || trust) {
                        // Pass vision measurement back through consumer
                        visionConsumer.accept(new VisionMeasurement(this.llPose, poseEstimate.timestampSeconds, stdDevs));
                    } else {
                        distanceError++;
                        SmartDashboard.putNumber(llName + "/Limelight Error", distanceError);
                    }
                } else {
                    fieldError++;
                    SmartDashboard.putNumber(llName + "/Field Error", fieldError);
                }
            }
        }
    }

    private boolean isPoseWithinFieldBoundaries(Pose2d pose) {
        return pose.getX() >= 0.0 && pose.getX() <= aprilTagLayout.getFieldLength() &&
               pose.getY() >= 0.0 && pose.getY() <= aprilTagLayout.getFieldWidth();
    }

    private boolean isSpinningTooFast(double angularVelocity) {
        if (isLL4) {
            return false;
        } else {
            return Math.abs(angularVelocity) > 180;
        }
    }

    public void logTargets() {
        LimelightHelpers.RawFiducial[] targets = LimelightHelpers.getRawFiducials(llName);
        Pose3d[] poses = new Pose3d[targets.length];
        double[] ids = new double[targets.length];

        for (int i = 0; i < targets.length; i++) {
            ids[i] = targets[i].id;
            Optional<Pose3d> targetPose = aprilTagLayout.getTagPose((int) ids[i]);
            poses[i] = targetPose.orElseGet(Pose3d::new);
        }
    }

    public void useLimelight(boolean enable) {
        this.enable = enable;
    }

    public void trustLL(boolean trust) {
        this.trust = trust;
    }

    public void enable() {
        this.enable = true;
    }

    public void disable() {
        this.enable = false;
    }
}