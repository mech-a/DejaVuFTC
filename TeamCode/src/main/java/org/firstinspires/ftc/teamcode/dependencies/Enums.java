package org.firstinspires.ftc.teamcode.dependencies;

public class Enums {
    public enum GoldPosition {
        LEFT, MIDDLE, RIGHT, UNK
    }

    public enum OpModeType {
        TELEOP, AUTON
    }

    //TODO figure out enums for arrays

    public enum DriveMotor {
        FL(0), FR(1), BR(2), BL(3);

        private int index;

        DriveMotor(int i) { index = i; }

        public int getIndex() { return index; }
    }

    public enum ArmMotor {
        RAISE(0), TELESCOPE(1), ROTATE(2), INTAKE(3);

        private int index;

        ArmMotor(int i) { index = i; }

        public int getIndex() { return index; }
    }
}
