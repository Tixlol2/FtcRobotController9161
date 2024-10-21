package org.firstinspires.ftc.teamcode.overload;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;


@TeleOp(name="servoTest", group = "Drive")
public class servoTest extends LinearOpMode {

    //Class def


    Servo servo;


    @Override
    public void runOpMode() throws InterruptedException {

        //ll3a = hardwareMap.get(Limelight3A.class, "LL3a");
        servo = hardwareMap.get(Servo.class, "Max");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();
        //During Initialization:


        while (opModeIsActive()) {

            if (gamepad2.b) {
                servo.setPosition(.4);
            } else if (gamepad2.a) {
                servo.setPosition(1);
            }

        }
    }
}
