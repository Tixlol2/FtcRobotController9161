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
    public final DcMotorEx extenderMotor;
    public final DcMotorEx angleMotor;


    private int targetDG;
    private int targetTK;

    private int extTargetIN;
    private int extTargetTK;




    private final double ticks_in_degree = (751.8 / 3) / 360;

    private final double ticks_in_inch = ticks_in_degree / 4.409;


    public armSubsystem(final HardwareMap hmap, final String extension, final String angle){
        extenderMotor = hmap.get(DcMotorEx.class, extension);
        angleMotor = hmap.get(DcMotorEx.class, angle);
    }



    public void setArmAngle(int degrees){
        targetDG = degrees;
        targetTK = (int) (targetDG / ticks_in_degree);
    }

    public int getAngleTargetTK(){
        return targetTK;
    }
    public int getAngleTargetDG(){return targetDG;}

    public void setExtendTarget(int inches){
        extTargetIN = inches;
        extTargetTK = (int) (extTargetIN * ticks_in_inch);


    }

    public int getExtTargetIN(){return extTargetIN;}
    public int getExtTargetTK(){return extTargetTK;}

    public int getExtenderPos(){return extenderMotor.getCurrentPosition();}
    public int getAnglePos(){return angleMotor.getCurrentPosition();}

    public int getExtenderPosIN(){return (int) (extenderMotor.getCurrentPosition() * ticks_in_inch);}
    public int getAnglePosDEG(){return (int) (angleMotor.getCurrentPosition() * ticks_in_degree);}

}
