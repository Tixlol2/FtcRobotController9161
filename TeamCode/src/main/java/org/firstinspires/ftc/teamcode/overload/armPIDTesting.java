package org.firstinspires.ftc.teamcode.overload;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class armPIDTesting extends LinearOpMode {

    private PIDController controller;

    public static double p = 0.0028, i = 0.05, d = 0.0002;
    public static double f = -.15;

    public static int target = 0;

    private final double ticks_in_degree = (751.8 * 4) / 360;

    private final double ticks_in_inch = ticks_in_degree / 4.409;

    private DcMotorEx arm_motor;


    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm_motor = hardwareMap.get(DcMotorEx.class, "armAng");



        waitForStart();

        while(!isStopRequested()){

            //controller.setPID(p, i, d);
            int armPos = arm_motor.getCurrentPosition();
            double pidPower = controller.calculate(armPos, target);
            double feedForward = Math.cos(Math.toRadians(armPos / ticks_in_degree)) * f;

            double power = pidPower + feedForward;

            arm_motor.setPower(power);

            telemetry.addData("Current Pos: ", armPos);
            telemetry.addData("Current Target: ", target);

            telemetry.update();



        }



    }
}
