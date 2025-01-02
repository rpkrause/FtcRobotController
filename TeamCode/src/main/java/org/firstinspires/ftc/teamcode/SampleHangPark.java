package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.VerticalArm;

@Autonomous(name="Sample Hang Park", group = "opMode")
public class SampleHangPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VerticalArm vArm = new VerticalArm(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        // close hand to start
        vArm.closeHand();

        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);

        // left
        Trajectory trajectoryLeft = drive.trajectoryBuilder(startPose)
                .strafeLeft(24)
                .build();
        drive.followTrajectory(trajectoryLeft);

        // forward
        Trajectory trajectoryForward = drive.trajectoryBuilder(trajectoryLeft.end())
                .forward(24)
                .build();
        drive.followTrajectory(trajectoryForward);

        // raise arm
        vArm.moveToHeight(26);
        sleep(2000);

        // move forward more
        trajectoryForward = drive.trajectoryBuilder(trajectoryForward.end())
                .forward(6)
                .build();
        drive.followTrajectory(trajectoryForward);

        // lower arm to hook
        vArm.moveToHeight(22.5);
        sleep(1000);
        vArm.openHand();
        sleep(500);

        // back
        Trajectory trajectoryBack = drive.trajectoryBuilder(trajectoryForward.end())
                .back(6)
                .build();
        drive.followTrajectory(trajectoryBack);

        // lower arm
        vArm.moveToHeight(0);

        // back more
        trajectoryBack = drive.trajectoryBuilder(trajectoryBack.end())
                .back(24)
                .build();
        drive.followTrajectory(trajectoryBack);

        // right
        Trajectory trajectoryRight = drive.trajectoryBuilder(trajectoryBack.end())
                .strafeRight(38)
                .build();
        drive.followTrajectory(trajectoryRight);
    }
}
