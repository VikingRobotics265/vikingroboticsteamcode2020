package org.firstinspires.ftc.teamcode.vikingroboticsteamcode2020.Fall2020.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BuildConfig;

abstract public class PID extends LinearOpMode {
    protected double kP, kI, kD;
    protected double tolerance;
    protected double prevErr, sumErr;
    protected final long DELTA_TIME = 10;
    public PID(double kP, double kI, double kD, double tolerance) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.tolerance = tolerance;
    }
    abstract void control(double output);
    abstract double read();
    abstract void end();
    public double err(double tar) {
        return tar - read();
    }
    public double evalOnce(double tar) { // side
        double err0 = err(tar);
        double deltaErr = err0 - prevErr;
        prevErr = err0;
        sumErr += err0;
        return kP*err0 + kI*sumErr + kD*deltaErr;
    }
    public void runOnce(double tar) {
        control(evalOnce(tar));
    }
    public void run(double tar) {
        while(opModeIsActive() && err(tar) < tolerance) {
            runOnce(tar);
        }
        end();
    }
    @Override public void runOpMode() {}
}
abstract class BoundedPID extends PID {
    protected double infOut, supOut;
    public BoundedPID(double kP, double kI, double kD, double tolerance,
                      double infOut, double supOut) throws AssertionError {
        super(kP, kI, kD, tolerance);
        if (BuildConfig.DEBUG && infOut > supOut)
            throw new AssertionError("Assertion failed: (infOut > supOut) is absurd");
        this.infOut = infOut;
        this.supOut = supOut;
    }
    public double evalOnce(double tar) {
        return Range.clip(super.evalOnce(tar), infOut, supOut);
    }
}
abstract class CyclicPID extends BoundedPID {
    protected double infIn, supIn;
    public CyclicPID(double kP, double kI, double kD, double tolerance,
                     double infOut, double supOut, double infIn, double supIn) throws AssertionError{
        super(kP, kI, kD, tolerance, infOut, supOut);
        if (BuildConfig.DEBUG && infIn > supIn)
            throw new AssertionError("Assertion failed: (infIn > supIn) is absurd");
        this.infIn = infIn;
        this.supIn = supIn;
    }
    public double err(double tar) {
        double err = super.err(tar);
        double rangeIn = supIn - infIn;
        err += (2*err > rangeIn)? (err > 0)? -rangeIn: rangeIn: 0;
        return err;
    }
}
