package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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



    public  double pAngle = 0.0045, iAngle = 0.05, dAngle = 0, fAngle = 0;

    public  int angleTarget = 0;



    private int armAngle;



    private final double ticks_in_degree = (751.8 * 3) / 360;

    private final double ticks_in_inch = 537.7 / (112 / 25.4);



    public  double pExtend = 0.008, iExtend = 0.05, dExtend = 0, fExtend = 0;

    public  int target_in_ticksExtend = 0;





    public armSubsystem(final HardwareMap hmap, final String extension, final String angle){
        extenderMotor = hmap.get(DcMotorEx.class, extension);
        angleMotor = hmap.get(DcMotorEx.class, angle);

    }

    // ----------------
    // Setters
    // ----------------

    public void setArmAngle(int degrees){
        targetDG = degrees;
        targetTK = (int) (targetDG / ticks_in_degree);
    }

    public void setArmTarget(int ticks) {
        targetTK = ticks;
    }


    // ----------------
    // Getters
    // ----------------
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

    // ----------------
    // Kinematics
    // ----------------

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


    // ----------------
    // Calculations
    // ----------------



    public void motorCalculations(int angleTarget, int extendTarget, PIDController angleController, PIDController extendController) {
        double anglePower;
        double extendPower;
        double anglefeedForward;
        double anglePIDFpower;
        int extendPos;


        double anglePIDFPower;

        if (angleTarget >= -15) {
            angleTarget = -15;
        } else if (angleTarget <= -650) {
            angleTarget = -650;
        }

        if (extendTarget >= -50) {
            extendTarget = -50;
        } else if (extendTarget <= -3400) {
            extendTarget = -3400;
        }
//        } else if (Math.cos(Math.toRadians(armAngle / ticks_in_degree)) * extendTarget >= (40 - 18) * ticks_in_inch) {
//            extendTarget = (int) (Math.cos(Math.toRadians(armAngle / ticks_in_degree)) * (40 - 18) * ticks_in_inch);
//        }
        //Angle motor
        angleController.setPID(pAngle, iAngle, dAngle);
        armAngle = angleMotor.getCurrentPosition();
        anglePIDFpower = angleController.calculate(armAngle, angleTarget);
        anglefeedForward = Math.cos(Math.toRadians(armAngle / ticks_in_degree)) * fAngle;
        anglePower = anglePIDFpower + anglefeedForward;
        if (anglePower > .8) {
            anglePower = .8;
        } else if (anglePower < -.8) {
            anglePower = -.8;
        }
        angleMotor.setPower(anglePower);

        //Extension motor
        extendController.setPID(pExtend, iExtend, dExtend);
        extendPos = extenderMotor.getCurrentPosition();
        extendPower = extendController.calculate(extendPos, extendTarget);

        if (extendPower > .8) {
            extendPower = .8;
        } else if (extendPower < -.8) {
            extendPower = -.8;
        }
        extenderMotor.setPower(extendPower);

        //return new int[] {angleTarget, extendTarget};
    }
}
