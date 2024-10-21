package org.firstinspires.ftc.teamcode.overload;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;


@TeleOp(name="Pedro Drive", group = "Drive")
public class pedroDrive extends LinearOpMode {

    //Class def


    double deflator;



    //Limelight3A ll3a;
    Follower follower;
    boolean driveCentric;



    @Override
    public void runOpMode() throws InterruptedException {

        //ll3a = hardwareMap.get(Limelight3A.class, "LL3a");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //hMap, name of servo used for claw
        clawSubsystem clawSubsystem = new clawSubsystem(hardwareMap, "clawAngle", "clawDriver");
        //hMap, name of motor used to change the EXTENSION HEIGHT of the arm/slides
        armSubsystem armSubsystem = new armSubsystem(hardwareMap, "armExt", "armAng");

        follower = new Follower(hardwareMap);

        waitForStart();
        //During Initialization:

        follower.startTeleopDrive();



        while (opModeIsActive()) {


            //Drive
            // ----------------------------
            deflator = gamepad1.left_bumper && gamepad1.right_bumper ? 0.5 : gamepad1.left_bumper ? 0.7 : 1;
            if (gamepad1.b) {
                driveCentric = false;
            } else if (gamepad1.a) {
                driveCentric = true;
            }


            // ----------------------------
            // Other Funcs Used in TeleOp
            // ----------------------------

            //Testing clawSubsystem
            if (gamepad2.b) {
                clawSubsystem.open();
            } else if (gamepad2.a) {
                clawSubsystem.close();
            }
            //Testing extendSubsystem





            armSubsystem.extenderMotor.setPower(gamepad2.right_stick_y);




            // ----------------------------
            // Updaters
            // ----------------------------




            telemetry.addData("Current TK Pos of Angle: ", armSubsystem.getExtenderPos());
            telemetry.addData("Current DEG Pos of Angle: ", armSubsystem.getAngleTargetDG());


            telemetry.addData("Current TK Pos of Extension: ", armSubsystem.getAnglePos());
            telemetry.addData("Current INCH Pos of Extension: ", armSubsystem.getExtenderPosIN());


            telemetry.addLine("Don't Crash!");
            telemetry.addData("Driver Centric?", driveCentric);


            telemetry.update();

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, driveCentric);
            follower.update();
            // ----------------------------
        }

    }
}
