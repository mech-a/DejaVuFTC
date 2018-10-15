package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

//TODO see if throw exception clause?
public class Robot {
    //TODO based on specification, add more motor slots

    public DcMotor[] drive_motors = new DcMotor[4];
    public BNO055IMU imu;
    //public VuforiaLocalizer vuf;
    public double[] drive_mtr_powers = new double[4];

    LinearOpMode caller;

    public Robot(LinearOpMode initializer) {
        caller = initializer;
    }


}
