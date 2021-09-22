package frc.robot;

public class util {
    public static double deadband(double input, double width) {
            return (Math.abs(input)<(width/2.0)) ? 0.0 : input;
    }
}
