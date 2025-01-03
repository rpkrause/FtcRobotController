package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.VerticalArm;

@Autonomous(name="Multiple Sample Hang", group = "opMode")
public class MultiHang extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VerticalArm vArm = new VerticalArm(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        // close hand to start
        vArm.closeHand();

        //We start at x: 26, y:-55.5, heading: 90 degrees
        Pose2d startPose = new Pose2d(26, -55.5, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Raise Arn
        vArm.moveToHeight(26);

        sleep(750);

        //Drive to bar
        Trajectory trajectoryToBarInitial = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(0,-26), Math.toRadians(90))
                .build();
        drive.followTrajectory(trajectoryToBarInitial);

        sleep(500);

        //Hang Sample
        vArm.moveToHeight(22.5);
        sleep(500);
        vArm.openHand();
        vArm.moveToHeight(1.5);

        sleep(500);

        //Move to bring back sample
        Trajectory trajectoryretrieveSample1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(9,-30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36,-30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36,-7), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(49,-9), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(49,-42), Math.toRadians(90))
                .build();
        drive.followTrajectory(trajectoryretrieveSample1);

    }
}
