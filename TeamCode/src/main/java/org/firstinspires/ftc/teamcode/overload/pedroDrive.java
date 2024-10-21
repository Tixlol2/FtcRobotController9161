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

    private PIDController controller;
    double deflator;

    public static double p = .0035, i = .05, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = (751.8 / 3) / 360;

    private final double ticks_in_inch = ticks_in_degree / 4.409;


    //Limelight3A ll3a;
    Follower follower;
    boolean driveCentric;



    @Override
    public void runOpMode() throws InterruptedException {

        //ll3a = hardwareMap.get(Limelight3A.class, "LL3a");
        controller = new PIDController(p, i, d);
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


            target += (int) (Math.pow(gamepad2.left_stick_y, 3) * 8);


            armSubsystem.extenderMotor.setPower(gamepad2.right_stick_y);




            // ----------------------------
            // Updaters
            // ----------------------------


            controller.setPID(p, i, d);
            int armPos = armSubsystem.angleMotor.getCurrentPosition();
            double pidPower = controller.calculate(armPos, target);
            double feedForward = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pidPower + feedForward;

            armSubsystem.angleMotor.setPower(power);

            telemetry.addData("Current Pos: ", armPos);
            telemetry.addData("Current Target: ", target);

            telemetry.addLine("Don't Crash!");
            telemetry.addData("Driver Centric?", driveCentric);


            telemetry.update();

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, driveCentric);
            follower.update();
            // ----------------------------
        }

    }
}
