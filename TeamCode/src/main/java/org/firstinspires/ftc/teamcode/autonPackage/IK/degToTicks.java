package org.firstinspires.ftc.teamcode.autonPackage.IK;

public class degToTicks {

    public int degToTicksHD(double degrees) {
        if (degrees == 0) {
            return 0;
        }
        //double NDegrees = ;
        return (int)Math.floor(950 * (degrees / 360));
        //return 0;
    }
    public int degToTicksCore(double degrees) {
        if (degrees == 0) {
            return 0;
        }
        double NDegrees = degrees / 360;
        return (int)Math.floor(280.0 / NDegrees);
        //return 0;
    }

}
