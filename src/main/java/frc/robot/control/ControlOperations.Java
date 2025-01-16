public class ControlOperations {
    private ControlOperations() {}
    
    public static double deadBand(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
}