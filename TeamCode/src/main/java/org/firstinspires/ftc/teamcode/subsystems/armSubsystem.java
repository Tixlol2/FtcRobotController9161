package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private double armX;
    private double armY;

    private PIDController pidFController;

    public  double pAngle = 0.0035, iAngle = 0.05, dAngle = 0, fAngle = 0;

    public  int angleTarget = 0;



    private int armAngle;
    public double anglePower;
    private double anglePIDFpower;
    private double anglefeedForward;

    private final double ticks_in_degree = (751.8 * 3) / 360;

    private final double ticks_in_inch = 537.7 / (112 / 25.4);

    private PIDController pidController;

    public  double pExtend = 0.008, iExtend = 0.05, dExtend = 0, fExtend = 0;

    public  int target_in_ticksExtend = 0;


    private int extendPos;
    public double powerExtension;
    private double PIDFpowerExtend;




    public armSubsystem(final HardwareMap hmap, final String extension, final String angle){
        extenderMotor = hmap.get(DcMotorEx.class, extension);
        angleMotor = hmap.get(DcMotorEx.class, angle);

    }




    public void setArmAngle(int degrees){
        targetDG = degrees;
        targetTK = (int) (targetDG / ticks_in_degree);
    }

    public void setArmTarget(int ticks) {
        targetTK = ticks;
    }

    public int getAngleTargetTK(){
        return targetTK;
    }
    public int getAngleTargetDG(){return targetDG;}

    public void setExtendTarget(int ticks){
        extTargetTK = ticks;


    }

    public int getExtTargetIN(){return extTargetIN;}
    public int getExtTargetTK(){return extTargetTK;}

    public int getExtenderPos(){return extenderMotor.getCurrentPosition();}
    public int getAnglePos(){return angleMotor.getCurrentPosition();}

    public int getExtenderPosIN(){return (int) (extenderMotor.getCurrentPosition() / ticks_in_inch);}
    public int getAnglePosDEG(){return (int) (angleMotor.getCurrentPosition() / ticks_in_degree);}

    public void resetEncoders(){
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extenderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double[] getPosition() {
        armX = Math.cos(Math.toRadians(getAnglePosDEG())) * (getExtenderPosIN() - 18);
        armY = Math.sin(Math.toRadians(getAnglePosDEG())) * (getExtenderPosIN()-18);
        double[] position = {armX,armY};
        return position;
    }

    public void setPostion(double x, double y) {
        if (x <= -42) {x = -42;}
        else if (x >= -18) {x=-18;}
        if (y <= 0 ) {y=0;}

        setArmTarget((int) (Math.toDegrees(Math.atan(y/x)) * ticks_in_degree));
        setExtendTarget((int) ((int) -(Math.sqrt((x)*(x) + y*y)-18) * ticks_in_inch));

    }

}
