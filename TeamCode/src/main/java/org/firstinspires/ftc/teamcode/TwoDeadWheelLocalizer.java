package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {

    public static class Params {
        /** forward/back offset of parallel wheel (ticks) */
        public double forwardOffsetTicks = -7670;

        /** left/right offset of perpendicular wheel (ticks) */
        public double lateralOffsetTicks = -3175;
    }

    public static Params PARAMS = new Params();

    private final Encoder par, perp;
    private final IMU imu;
    private final double inPerTick;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;
    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    private Pose2d pose;
    private Pose2d spinStartPose = new Pose2d(0, 0, 0);

    public TwoDeadWheelLocalizer(
            HardwareMap hardwareMap,
            IMU imu,
            double inPerTick,
            Pose2d initialPose
    ) {
        par = new OverflowEncoder(new RawEncoder(
                hardwareMap.get(DcMotorEx.class, "par")));
        perp = new OverflowEncoder(new RawEncoder(
                hardwareMap.get(DcMotorEx.class, "perp")));

        this.imu = imu;
        this.inPerTick = inPerTick;
        this.pose = initialPose;
    }

    /* ===================== TUNING HELPERS ===================== */

    /** Call before each spin */
    public void resetForSpin(Pose2d pose) {
        setPose(pose);
        initialized = false;
    }

    /** Mark start of a spin */
    public void markSpinStart() {
        spinStartPose = pose;
    }

    /** X/Y drift accumulated during a spin */
    public Vector2d getSpinDelta() {
        Pose2d d = pose.minus(spinStartPose);
        return new Vector2d(d.position.x, d.position.y);
    }

    public void setOffsets(double forwardInches, double lateralInches) {
        PARAMS.forwardOffsetTicks = forwardInches / inPerTick;
        PARAMS.lateralOffsetTicks = lateralInches / inPerTick;
    }

    /* ========================================================== */

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair parPV = par.getPositionAndVelocity();
        PositionVelocityPair perpPV = perp.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        AngularVelocity avDeg = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        AngularVelocity av = new AngularVelocity(
                UnnormalizedAngleUnit.RADIANS,
                (float) Math.toRadians(avDeg.xRotationRate),
                (float) Math.toRadians(avDeg.yRotationRate),
                (float) Math.toRadians(avDeg.zRotationRate),
                avDeg.acquisitionTime
        );

        Rotation2d heading = Rotation2d.exp(
                angles.getYaw(AngleUnit.RADIANS));

        double rawHeadingVel = av.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;
            lastParPos = parPV.position;
            lastPerpPos = perpPV.position;
            lastHeading = heading;
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }

        int dPar = parPV.position - lastParPos;
        int dPerp = perpPV.position - lastPerpPos;
        double dHeading = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[]{
                                dPar - PARAMS.forwardOffsetTicks * dHeading,
                                parPV.velocity - PARAMS.forwardOffsetTicks * headingVel
                        }).times(inPerTick),
                        new DualNum<>(new double[]{
                                dPerp - PARAMS.lateralOffsetTicks * dHeading,
                                perpPV.velocity - PARAMS.lateralOffsetTicks * headingVel
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[]{ dHeading, headingVel })
        );

        lastParPos = parPV.position;
        lastPerpPos = perpPV.position;
        lastHeading = heading;

        pose = pose.plus(twist.value());
        return twist.velocity().value();
    }

}
