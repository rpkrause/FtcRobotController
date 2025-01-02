package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.HorizontalArm;

@Autonomous(name = "Horizontal Arm Test", group = "Test")
public class HorizontalArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        final long defaultWait = 1000;

        HorizontalArm arm = new HorizontalArm(hardwareMap);

        waitForStart();

        arm.openHand();
        sleep(defaultWait);
        arm.closeHand();
        sleep(defaultWait);
        arm.openHand();
        sleep(defaultWait);

        arm.armExtensionDistance(13);
        sleep(defaultWait);
        arm.armExtensionDistance(1);
        sleep(defaultWait);
        arm.rotateHandDown();
        sleep(defaultWait);
        arm.rotateHandUp();
        sleep(defaultWait);

        arm.openHand();
        sleep(defaultWait);
        arm.closeHand();
        sleep(defaultWait);
        arm.openHand();
        sleep(defaultWait);
    }
}