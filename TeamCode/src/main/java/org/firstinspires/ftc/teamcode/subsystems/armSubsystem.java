package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;

public class armSubsystem extends SubsystemBase {

    //Motor used to change the angle of the arm
    private final DcMotorEx extenderMotor;
    private final DcMotorEx angleMotor;


    private PIDFController angleController;
    private PIDController extendController;

    //These values will be set via armPIDTesting once tuned, due to it being the exact same PIDF lol
    public static double pA = 0, iA = 0, dA = 0, fA = 0;
    //Should be the same with these values
    public static double pE = 0, iE = 0, dE = 0;

    //Arm angle PIDF local variables
    int armPos;
    double pidFPower;
    int targetTicks;
    int errorF;
    //Extension PID local variables
    int extendPos;
    double powerE;




    private final double ticks_in_degree = 537.7;




    public armSubsystem(final HardwareMap hmap, final String extension, final String angle){
        extenderMotor = hmap.get(DcMotorEx.class, extension);
        angleMotor = hmap.get(DcMotorEx.class, angle);
    }

    public void extendIn(int inches){
        //TODO: Some conversion of an input in INCHES to TICKS so that the motor, extender, lifts the arm system up INCHES.
        //this works so well 100% (its 2am)
        extenderMotor.setTargetPosition(inches);

    }

    public void setArmAngle(int degrees){
        angleController.setPIDF(pA, iA, dA, fA);
        targetTicks = (int) Math.round((degrees * ticks_in_degree));


        //Theoretical PIDF goes hard
        while(errorF != 0) {
            armPos = angleMotor.getCurrentPosition();
            pidFPower = angleController.calculate(armPos, targetTicks);
            errorF = targetTicks - armPos;
            angleMotor.setPower(pidFPower);
        }

    }


}
