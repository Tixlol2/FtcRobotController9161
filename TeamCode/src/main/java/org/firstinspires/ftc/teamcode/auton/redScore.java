package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class redScore extends LinearOpMode {


    Pose start = new Pose(36, 12, 0);
    Pose middleOf3Spec = new Pose(12, 32, 0);
    Pose faceLeftSpec = new Pose(12, 32, Math.toRadians(45));
    Pose faceMiddleSpec = new Pose(12, 32, 0);
    Pose faceRightSpec = new Pose(12, 32, -Math.toRadians(45));

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        //During Init





        while(!isStopRequested()){
            //Main Function






        }








    }
}
