package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pose Test")
public class PoseTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize your mecanum drive and localizer
        // Start pose can be (0,0,0) or any calibrated starting pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Ready! Press play to start pose test.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update pose estimate
            drive.updatePoseEstimate();

            // Get current estimated pose
            Pose2d pose = drive.localizer.getPose();

            // Display pose info on telemetry
            telemetry.addData("x (in)", pose.position.x);
            telemetry.addData("y (in)", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }
    }
}
