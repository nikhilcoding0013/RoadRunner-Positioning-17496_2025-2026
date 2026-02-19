package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "TwoWheel Offset Tuner")
public class TwoWheelOffsetTuner extends LinearOpMode {
    public static double fwdIN = -4.043;
    public static double latIN = -0.927;
    
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        TwoDeadWheelLocalizer localizer = (TwoDeadWheelLocalizer) drive.localizer;
        localizer.setOffsets(fwdIN, latIN);
        telemetry.addLine("Ready. Press Play to spin.");
        telemetry.update();
        waitForStart();

        drive.updatePoseEstimate();

        while (opModeIsActive()) {

            // Spin the robot
            drive.leftFront.setPower(-0.35);
            drive.leftBack.setPower(-0.35);
            drive.rightFront.setPower(0.35);
            drive.rightBack.setPower(0.35);

            drive.updatePoseEstimate();

            double heading = localizer.getPose().heading.toDouble();

            // Stop
            if (heading > 1.525) {
                break;
            }

            telemetry.addData("X (in)", localizer.getPose().position.x);
            telemetry.addData("Y (in)", localizer.getPose().position.y);
            telemetry.addData("Heading (rad)", heading);
            telemetry.update();
        }

        // Stop the motors
        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightBack.setPower(0);

        drive.updatePoseEstimate();
        Pose2d finalPose = localizer.getPose();

        //double lateralOffsetInches = 4* finalPose.position.x / (2 * Math.PI);
        //double forwardOffsetInches = 4* finalPose.position.y / (2 * Math.PI);

        while (opModeIsActive()) {
            telemetry.addLine("=== TUNING COMPLETE ===");
            telemetry.addData("finalX", finalPose.position.x);
            telemetry.addData("finalY", finalPose.position.y);
            //telemetry.addData("lateralOffset", lateralOffsetInches);
            //telemetry.addData("forwardOffset", forwardOffsetInches);
            telemetry.update();
        }
    }
}
