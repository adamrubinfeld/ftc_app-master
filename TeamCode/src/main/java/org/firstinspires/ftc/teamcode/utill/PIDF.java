package org.firstinspires.ftc.teamcode.utill;

public class PIDF {
    public double Kp;
    public double Ki;
    public double Kd;
    public double Kf;
    public double Izone;

    public double integral = 0;
    public double lastError = 0;
    public double lastTime = 0;


    public PIDF(double Kp, double Ki, double Kd, double Kf, double Izone){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.Izone = Izone;
    }

    public double PID(double target, double current, double currentTime){
        double error = target - current;
        integral += error;
        double derivative = (current - lastError)/(currentTime - lastTime);



        if (current < Izone){
            Ki = 0;
        }

        double P = error*Kp;
        double I = integral*Ki;
        double D = derivative*Kd;

        lastError = error;
        lastTime = currentTime;
        return  P+I+D;
    }

    public double PIDF(double target, double current, double currentTime){
        double error = target - current;
        integral += error;
        double derivative = (current - lastError)/(currentTime - lastTime);



        if (current < Izone){
            Ki = 0;
        }

        double P = error*Kp;
        double I = integral*Ki;
        double D = derivative*Kd;
        double F = target*Kf;

        lastError = error;
        lastTime = currentTime;
        return  P+I+D+F;
    }
}
