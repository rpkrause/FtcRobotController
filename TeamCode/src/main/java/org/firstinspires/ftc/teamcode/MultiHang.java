package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.VerticalArm;

@Autonomous(name="Two Sample Hang", group = "opMode")
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
        vArm.moveToHeight(27);

        sleep(1000);

        //Drive to bar
        Trajectory trajectoryToBar0 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(0,-30), Math.toRadians(90))
                .build();
        drive.followTrajectory(trajectoryToBar0);

        sleep(100);

        //Complete movement to the bar
        Trajectory trajectoryToBarComplete0 = drive.trajectoryBuilder(trajectoryToBar0.end())
                .forward(4.5)
                .build();
        drive.followTrajectory(trajectoryToBarComplete0);

        sleep(100);

        //Hang Specimen
        vArm.moveToHeight(22);
        sleep(500);
        vArm.openHand();
        vArm.moveToHeight(1.5);

        sleep(100);

        //Move back from the bar
        Trajectory trajectoryBackFromBar0 = drive.trajectoryBuilder(trajectoryToBarComplete0.end())
                .back(12)
                .build();
        drive.followTrajectory(trajectoryBackFromBar0);

        //Move right from the bar
        Trajectory trajectoryRightBar0 = drive.trajectoryBuilder(trajectoryBackFromBar0.end())
                .strafeRight(40)
                .build();
        drive.followTrajectory(trajectoryRightBar0);

        //Move in front of sample 1
        Trajectory trajectoryPrimeSample1 = drive.trajectoryBuilder(trajectoryRightBar0.end())
                .splineToConstantHeading(new Vector2d(40,-8), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(50,-4), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(trajectoryPrimeSample1);

        //Retrieve sample 1
        Trajectory trajectoryRetrieveSample1 = drive.trajectoryBuilder(trajectoryPrimeSample1.end())
                .lineToConstantHeading(new Vector2d(50,-42))
                .build();
        drive.followTrajectory(trajectoryRetrieveSample1);

        //Move next to sample 2 (HUMAN PLAYER MUST ACT IN THIS PERIOD)
        Trajectory trajectoryPrimeSample2 = drive.trajectoryBuilder(trajectoryRetrieveSample1.end())
                .splineToConstantHeading(new Vector2d(50,-8), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60,-4), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(trajectoryPrimeSample2);

        vArm.moveToHeight(11);

        //Retrieve sample 2
        Trajectory trajectoryRetrieveSample2 = drive.trajectoryBuilder(trajectoryPrimeSample2.end())
                .lineToConstantHeading(new Vector2d(60,-48))
                .build();
        drive.followTrajectory(trajectoryRetrieveSample2);

        //Align specimen 1
        Trajectory trajectoryAlignSpecimen1 = drive.trajectoryBuilder(trajectoryRetrieveSample2.end())
                .splineTo(new Vector2d(50,-46), Math.toRadians(270))
                .build();
        drive.followTrajectory(trajectoryAlignSpecimen1);

        sleep(100);

        //Take specimen 1
        Trajectory trajectoryTakeSpecimen1 = drive.trajectoryBuilder(trajectoryAlignSpecimen1.end())
                .lineToConstantHeading(new Vector2d(50,-54.5))
                .build();
        drive.followTrajectory(trajectoryTakeSpecimen1);

        sleep(250);

        vArm.closeHand();
        sleep(250);
        vArm.moveToHeight(27);
        sleep(750);

        //Back away from wall
        Trajectory trajectoryBackFromSpecimen = drive.trajectoryBuilder(trajectoryTakeSpecimen1.end())
                .back(18)
                        .build();
        drive.followTrajectory(trajectoryBackFromSpecimen);

        //Drive to the bar and spin
        Trajectory trajectoryToBar1 = drive.trajectoryBuilder(trajectoryBackFromSpecimen.end())
                .splineTo(new Vector2d(3,-30), Math.toRadians(90))
                .build();
        drive.followTrajectory(trajectoryToBar1);

        //Complete motion to bar
        Trajectory trajectoryToBarComplete1 = drive.trajectoryBuilder(trajectoryToBar1.end())
                .forward(4)
                .build();
        drive.followTrajectory(trajectoryToBarComplete1);

        //Hang the specimen
        sleep(100);

        //Hang Specimen
        vArm.moveToHeight(22.5);
        sleep(500);
        vArm.openHand();
        vArm.moveToHeight(1.5);

        sleep(100);

        //Move back from the bar
        Trajectory trajectoryBackFromBar1 = drive.trajectoryBuilder(trajectoryToBarComplete0.end())
                .back(4)
                .build();
        drive.followTrajectory(trajectoryBackFromBar1);

        sleep(1000);
    }
}
