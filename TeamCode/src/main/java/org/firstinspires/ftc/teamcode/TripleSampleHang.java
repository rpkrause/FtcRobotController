package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.VerticalArm;
import org.firstinspires.ftc.teamcode.util.HorizontalArm;

@Autonomous(name="Three Sample Hang", group = "opMode")
public class TripleSampleHang extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VerticalArm vArm = new VerticalArm(hardwareMap);
        HorizontalArm hArm = new HorizontalArm(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        // close hand to start
        vArm.closeHand();

        //We start at x: 26, y:-55.5, heading: 90 degrees
        Pose2d startPose = new Pose2d(26, -55.5, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Raise Arn
        vArm.moveToHeight(27);

        sleep(400);

        //Drive to bar
        Trajectory trajectoryToBar0 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(5,-30))
                .build();
        drive.followTrajectory(trajectoryToBar0);

        //Complete movement to the bar
        Trajectory trajectoryToBarComplete0 = drive.trajectoryBuilder(trajectoryToBar0.end())
                .forward(5)
                .build();
        drive.followTrajectory(trajectoryToBarComplete0);

        //Hang Specimen
        vArm.moveToHeight(22);
        sleep(300);
        vArm.openHand();
        vArm.moveToHeight(1.5);

        hArm.moveToExtensionDistance(18);

        //Move right from the bar
        Trajectory trajectorySample1Grab = drive.trajectoryBuilder(trajectoryToBarComplete0.end(),true)
                .splineTo(new Vector2d(31, -40), Math.toRadians(50))
                .addDisplacementMarker(hArm::rotateHandDown)
                .build();
        drive.followTrajectory(trajectorySample1Grab);

        //Grab sample 1
        sleep(500);
        hArm.closeHand();
        sleep(250);
        hArm.rotateHandUp();

        //Rotate robot to place in observatory zone & closer to second specimen
        Trajectory trajectorySample2Grab = drive.trajectoryBuilder(trajectorySample1Grab.end(), true)
                .splineTo(new Vector2d(33,-40), Math.toRadians(320))

                .addDisplacementMarker(hArm::openHand)
                .splineTo(new Vector2d(41, -40), Math.toRadians(50))
                .build();
        drive.followTrajectory(trajectorySample2Grab);

        //Grab the second sample
        hArm.rotateHandDown();
        sleep(500);
        hArm.closeHand();
        sleep(250);
        hArm.rotateHandUp();
        hArm.moveToExtensionDistance(10);

        //Rotate robot to place in observatory & get closer to where I need to grab
        Trajectory trajectoryObservatorySample2 = drive.trajectoryBuilder(trajectorySample2Grab.end(), true)
                .splineTo(new Vector2d(43,-38), Math.toRadians(320))
                .build();
        drive.followTrajectory(trajectoryObservatorySample2);

        hArm.openHand();
        hArm.moveToExtensionDistance(0);
        vArm.moveToHeight(11);

        //Align specimen 1
        Trajectory trajectoryAlignSpecimen1 = drive.trajectoryBuilder(trajectoryObservatorySample2.end())
                .splineTo(new Vector2d(50,-50), Math.toRadians(270))
                .build();
        drive.followTrajectory(trajectoryAlignSpecimen1);

        //Take specimen 1
        Trajectory trajectoryTakeSpecimen1 = drive.trajectoryBuilder(trajectoryAlignSpecimen1.end())
                .lineToConstantHeading(new Vector2d(50,-55))
                .build();
        drive.followTrajectory(trajectoryTakeSpecimen1);

        vArm.closeHand();
        sleep(150);
        vArm.moveToHeight(27);

//        //Back away from wall
//        Trajectory trajectoryBackFromSpecimen1 = drive.trajectoryBuilder(trajectoryTakeSpecimen1.end())
//                .back(18)
//                .build();
//        drive.followTrajectory(trajectoryBackFromSpecimen1);

        //Drive to the bar and spin
        Trajectory trajectoryToBar1 = drive.trajectoryBuilder(trajectoryAlignSpecimen1.end(), true)
                .splineTo(new Vector2d(30,-40), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(8,-30), Math.toRadians(270))
                .build();
        drive.followTrajectory(trajectoryToBar1);

        //Complete motion to bar
        Trajectory trajectoryToBarComplete1 = drive.trajectoryBuilder(trajectoryToBar1.end())
                .forward(5)
                .build();
        drive.followTrajectory(trajectoryToBarComplete1);

        //Hang Specimen
        vArm.moveToHeight(22.5);
        sleep(300);
        vArm.openHand();
        vArm.moveToHeight(11);

        //Move back from the bar
        Trajectory trajectoryBackFromBar1 = drive.trajectoryBuilder(trajectoryToBarComplete1.end())
                .back(4)
                .build();
        drive.followTrajectory(trajectoryBackFromBar1);

        //Move to grab specimen 2
        Trajectory trajectoryAlignSpecimen2 = drive.trajectoryBuilder(trajectoryToBarComplete1.end(), true)
                .splineTo(new Vector2d(50,-40), Math.toRadians(90))
                .build();
        drive.followTrajectory(trajectoryAlignSpecimen2);

        //Take specimen 1
        Trajectory trajectoryTakeSpecimen2 = drive.trajectoryBuilder(trajectoryAlignSpecimen2.end())
                .lineToConstantHeading(new Vector2d(50,-55))
                .build();
        drive.followTrajectory(trajectoryTakeSpecimen2);

        vArm.closeHand();
        sleep(150);
        vArm.moveToHeight(27);

//        //Back away from wall
//        Trajectory trajectoryBackFromSpecimen2 = drive.trajectoryBuilder(trajectoryTakeSpecimen2.end())
//                .back(18)
//                .build();
//        drive.followTrajectory(trajectoryBackFromSpecimen2);

        //Drive to the bar and spin
        Trajectory trajectoryToBar2 = drive.trajectoryBuilder(trajectoryAlignSpecimen2.end(), true)
                .splineTo(new Vector2d(30,-40), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(11,-30), Math.toRadians(270))
                .build();
        drive.followTrajectory(trajectoryToBar2);

        //Complete motion to bar
        Trajectory trajectoryToBarComplete2 = drive.trajectoryBuilder(trajectoryToBar2.end())
                .forward(5)
                .build();
        drive.followTrajectory(trajectoryToBarComplete2);

        //Hang Specimen
        vArm.moveToHeight(22.5);
        sleep(300);
        vArm.openHand();
        vArm.moveToHeight(1.5);

        Trajectory trajectoryBackFromBar2 = drive.trajectoryBuilder(trajectoryToBarComplete2.end())
                .back(4)
                .build();
        drive.followTrajectory(trajectoryBackFromBar2);
    }
}
