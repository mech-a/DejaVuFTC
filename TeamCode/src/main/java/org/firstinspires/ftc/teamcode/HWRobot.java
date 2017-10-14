package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HWRobot
{
    /* Declare all motors, sensors, etc. */
    public DcMotor mtrFL, mtrFR, mtrBL, mtrBR;


    // Declare speeds and other vars
    public double powFL = 0;
    public double powFR = 0;
    public double powBL = 0;
    public double powBR = 0;


    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415) ;
    static final double     DISTANCE_MODIFIER       = 1.414 ;

    /* local OpMode members and objects */
    HardwareMap hwMap           =  null;

    // Initialize standard Hardware interfaces.
    public void init(HardwareMap ahwMap, DcMotor.RunMode mode) {

        // Save reference to Hardware map.
        hwMap = ahwMap;

        // Define and initialize hardware
        mtrFL = ahwMap.dcMotor.get("fl_drive");
        mtrFR = ahwMap.dcMotor.get("fr_drive");
        mtrBL = ahwMap.dcMotor.get("bl_drive");
        mtrBR = ahwMap.dcMotor.get("br_drive");

        // Set directions for motors.
        mtrFL.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setDirection(DcMotor.Direction.FORWARD);

        //zero power behavior
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set power for all motors.
        mtrFL.setPower(powFL);
        mtrFR.setPower(powFR);
        mtrBL.setPower(powBL);
        mtrBR.setPower(powBR);

        // Set all motors to run with given mode
        mtrFL.setMode(mode);
        mtrFR.setMode(mode);
        mtrBL.setMode(mode);
        mtrBR.setMode(mode);

    }




    public void mtrChangeMode(DcMotor.RunMode mode){
        mtrFL.setMode(mode);
        mtrFR.setMode(mode);
        mtrBL.setMode(mode);
        mtrBR.setMode(mode);
    }
    public void mtrSetSpeed(double speed){
        mtrFL.setPower(speed);
        mtrFR.setPower(speed);
        mtrBL.setPower(speed);
        mtrBR.setPower(speed);
    }

    //Function to translate on the field (encoders)- currently reconstructed
    //TODO make repitition into own functions
    public void translate(String dir, double speed, double inches, boolean active){
        double inchLocal;
        String dirLocal;
        int posFL, posFR, posBL, posBR;
        int a = 1;
        int b = 1;

        dirLocal = dir.toLowerCase();
        inchLocal = Math.floor(inches * DISTANCE_MODIFIER);

        if(dir == "fwd" || dir == "forward") {
            a = 1;
            b = 1;
        }
        else if(dir == "bk" || dir == "backwards") {
            a = -1;
            b = -1;
        }
        else if(dir == "left") {
            a = -1;
            b = 1;
        }
        else if (dir == "right") {
            a = 1;
            b = -1;
        }

        int countTarget;

        if(active) {
            countTarget = mtrFL.getCurrentPosition() + (int) (inchLocal * COUNTS_PER_INCH);
            posFL = a * countTarget;
            posFR = b * countTarget;
            posBL = b * countTarget;
            posBR = a * countTarget;

            mtrFL.setTargetPosition(posFL);
            mtrFR.setTargetPosition(posFR);
            mtrBL.setTargetPosition(posBL);
            mtrBR.setTargetPosition(posBR);

            mtrChangeMode(DcMotor.RunMode.RUN_TO_POSITION);

            mtrSetSpeed(speed);

            mtrSetSpeed(0);

            mtrChangeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }


    //Function to rotate on the field (encoders)
    public void rotate(){}







    /* redundant- moved to init
    HWRobot(DcMotor amtrFL, DcMotor amtrFR, DcMotor amtrBL, DcMotor amtrBR, DcMotor.RunMode mode) {
        //Saves a localization
        mtrFL = amtrFL;
        mtrFR = amtrFR;
        mtrBL = amtrBL;
        mtrBR = amtrBR;


    }
    */




}

