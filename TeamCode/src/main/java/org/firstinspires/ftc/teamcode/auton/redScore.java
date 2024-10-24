package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;


public class redScore extends LinearOpMode {


    public int pathState = 0;

    public PathChain getToMiddle;

    Pose start = new Pose(12, 108, 0);
    Pose middleSpec = new Pose(32, 132, 0);


    Follower follower;

    armSubsystem armSubsystem = new armSubsystem(hardwareMap, "armExt", "armAng");
    clawSubsystem clawSubsystem = new clawSubsystem(hardwareMap, "clawAngle", "clawDriver");

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        //During Init
        follower = new Follower(hardwareMap);
        follower.resetIMU();
        follower.setStartingPose(start);



        getToMiddle = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(middleSpec), new Point(middleSpec.getX() + .0001, middleSpec.getY() + .0001, 1), new Point(middleSpec))))
                .setConstantHeadingInterpolation(middleSpec.getHeading())
                .build();


        while(!isStopRequested()){
            //Main Function

            follower.update();

            autonomousPathUpdate();






        }


    }

    public void autonomousPathUpdate() {

        switch(pathState){
            case 0:
                follower.followPath(getToMiddle);
                //TODO: Ensure values are correct
                armSubsystem.setExtendTarget(8);
                clawSubsystem.open();
                //This should set the claw to be ready to pinch the spec
                clawSubsystem.setAnglePosition(0);
                //Close the claw around the spec
                clawSubsystem.close();
                //flips claw back up
                clawSubsystem.setAnglePosition(1);
                //Retracts the arm
                armSubsystem.setExtendTarget(-8);
                setPathState(1);
                break;
            case 1:
                //set the angle of the arm to be able to reach the top box
                armSubsystem.setArmAngle(120);
                //extend to reach the top box
                armSubsystem.setExtendTarget(48);
                //Should be able to just drop the spec?
                clawSubsystem.open();
                setPathState(2);
        }


    }

    public void setPathState(int state){
        pathState = state;
        autonomousPathUpdate();
    }

}
