package org.firstinspires.ftc.teamcode.overload;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.*;


@TeleOp(name="Drive", group = "Drive")
public class Drive extends LinearOpMode {

    //Class def


    double deflator;
    double magnitude;
    double theta;
    double x;
    double y;
    double angVel;
    double thetaOffset = 0;

    Follower follower;

    //Limelight3A ll3a;

    boolean driveCentric;

    int counter = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        //ll3a = hardwareMap.get(Limelight3A.class, "LL3a");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive mecDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        //hMap, name of servo used for claw
        clawSubsystem clawSubsystem = new clawSubsystem(hardwareMap, "clawAngle", "clawDriver");
        //hMap, name of motor used to change the EXTENSION HEIGHT of the arm/slides
        armSubsystem armSubsystem = new armSubsystem(hardwareMap, "armExt", "armAng");



        waitForStart();
        //During Initialization:





        while (opModeIsActive()) {
            mecDrive.updatePoseEstimate();

            //Drive
            // ----------------------------
            deflator = gamepad1.left_bumper && gamepad1.right_bumper ? 0.5 : gamepad1.left_bumper ? 0.7 : 1;
            if (gamepad1.b) {
                driveCentric = false;
            } else if (gamepad1.a) {
                driveCentric = true;
            }
            if (driveCentric) {
                //Driver Centric (The front of the driver is the front of the controller)

                magnitude = Math.sqrt(Math.pow(-gamepad1.left_stick_x, 2) + Math.pow(-gamepad1.left_stick_y, 2));

                theta = thetaOffset + (-gamepad1.left_stick_y <= 0 && gamepad1.left_stick_x < 0 ? Math.atan(gamepad1.left_stick_x / -gamepad1.left_stick_y) + mecDrive.pose.heading.toDouble() - Math.PI : -gamepad1.left_stick_y <= 0 ? Math.atan(gamepad1.left_stick_x / -gamepad1.left_stick_y) + mecDrive.pose.heading.toDouble() + Math.PI : Math.atan(gamepad1.left_stick_x / -gamepad1.left_stick_y) + mecDrive.pose.heading.toDouble());

                x = (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 ? 0 : Math.cos(theta)) * magnitude * deflator;
                y = gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 ? 0 : -Math.sin(theta) * magnitude * deflator;

                telemetry.addLine("MODE: Driver Centric | The front of the driver is the front of the controller");
            } else {
                //Bot Centric (The front of the robot is the front of the controller)
                x = -gamepad1.left_stick_y;
                y = -gamepad1.left_stick_x;

                telemetry.addLine("MODE: Bot Centric | The front of the robot is the front of the controller");
            }

            // ----------------------------
            // Other Funcs Used in TeleOp
            // ----------------------------

            //Testing clawSubsystem
            if (gamepad2.circle){
                clawSubsystem.open();
                sleep(500);
                clawSubsystem.close();
            }
            //Testing extendSubsystem
            if (gamepad2.square){
                armSubsystem.extendIn(20);
                sleep(500);
                armSubsystem.extendIn(0);
            }


            // ----------------------------
            // Updaters
            // ----------------------------
            angVel = -gamepad1.right_stick_x * deflator;

            telemetry.addLine("Don't Crash!");

            telemetry.update();

            mecDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), angVel));
            // ----------------------------
        }

    }
}
