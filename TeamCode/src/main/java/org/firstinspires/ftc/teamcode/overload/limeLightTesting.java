package org.firstinspires.ftc.teamcode.overload;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="LL3a Testing", group = "Drive")
public class limeLightTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A ll3a;

        waitForStart();
        ll3a = hardwareMap.get(Limelight3A.class, "LL3a");
        telemetry.addLine(String.valueOf(ll3a.isRunning()));
        telemetry.update();

    }
}
