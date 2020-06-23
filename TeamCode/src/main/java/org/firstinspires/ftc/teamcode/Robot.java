package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Robot {

    final int offset = 0;

    DcMotor[] motor = new DcMotor[3];//0-lf, 1-lb, 2-rf, 3-4b
    String[] motorName = {"lf", "lb", "rf", "rb"};

    BNO055IMU imu;

    public Robot(){}

    public void Initialize(HardwareMap hw){
        for(int i = 0; i<motor.length; i++){
            motor[i] = hw.get(DcMotor.class, motorName[i]);
            motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (i<2){
                motor[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }
        //gyro stuff
        imu = hw.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public void DriveByVector(double x, double y, double r) { //r stands for rotation
        motor[0].setPower(y + x + r);
        motor[1].setPower(y - x + r);
        motor[2].setPower(y - x - r);
        motor[3].setPower(y + x - r);
    }

    public void FieldCentric(double x, double y, double r) {
        double vectorAngle = WarpAngle(Math.atan2(y,x));
        double robotAngle = WarpAngle(GetAngle()+offset);
        double finalAngle = vectorAngle + robotAngle;

        double vectorLength = Math.hypot(x, y);

        double finalX = Math.cos(finalAngle)*vectorLength;
        double finalY = Math.sin(finalAngle)*vectorLength;

        DriveByVector(finalX, finalY, r);
    }

    private double WarpAngle(double angle){
        return (angle+360)%360;
    }

    public double GetAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
