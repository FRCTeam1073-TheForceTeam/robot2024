package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OI extends SubsystemBase
{
    public final boolean debug = true;


    public Joystick driverController;
    private boolean isCubeMode;

    public Debouncer parkingBrakeDebouncer = new Debouncer(0.05);
    public Debouncer xDriverButtonDebouncer = new Debouncer(0.05);
    public Debouncer yDriverButtonDebouncer = new Debouncer(0.05);
    public Debouncer aDriverButtonDebouncer = new Debouncer(0.05);
    public Debouncer bDriverButtonDebouncer = new Debouncer(0.05);
    public Debouncer menuDriverButtonDebouncer = new Debouncer(0.05);
    public Debouncer viewDriverButtonDebouncer = new Debouncer(0.5);
    public Debouncer fieldCentricDebouncer = new Debouncer(.05);

    /** Setting up which controller is which
     * Drive Controller is controller 0
     * 
     * Sets what zero is on Driver Controller
     */
    public OI() 
    {
        driverController = new Joystick(0);
        LEFT_X_ZERO = 0;
        LEFT_Y_ZERO = 0;
        RIGHT_X_ZERO = 0;
        RIGHT_Y_ZERO = 0;
        zeroDriverController();

    }

    public static void initPreferences() 
    {
  
    }

    /**
     * Sets the Driver's controller to zero when we Enable
     */
    public void onEnable() {
        zeroDriverController();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("OI/isCubeMode", isCubeMode);
    }

    //------- Driver Controller Joysticks Below

    /**Zero's out Driver Controller
     * @param value - Value to Clamp
     * @param low - The lower boundary to which to clamp value.
     * @param high - The highest boundary to which to clamp value.
    */

    public void zeroDriverController() {
        //Sets all the offsets to zero, then uses whatever value it returns as the new offset.
        LEFT_X_ZERO = 0;
        LEFT_Y_ZERO = 0;
        RIGHT_X_ZERO = 0;
        RIGHT_Y_ZERO = 0;
        LEFT_X_ZERO = getDriverLeftX();
        LEFT_Y_ZERO = getDriverLeftY();
        RIGHT_X_ZERO = getDriverRightX();
        RIGHT_Y_ZERO = getDriverRightY();
    }

    private final double LEFT_X_MIN = -1;
    private final double LEFT_X_MAX = 1;
    private double LEFT_X_ZERO = 0;
    public double getDriverLeftX() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(0) - (LEFT_X_MAX + LEFT_X_MIN) * 0.5) / (LEFT_X_MAX - LEFT_X_MIN) - LEFT_X_ZERO, -1, 1);
    }

    private final double LEFT_Y_MIN = -1;
    private final double LEFT_Y_MAX = 1;
    private double LEFT_Y_ZERO = 0;
    public double getDriverLeftY() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(1) - (LEFT_Y_MAX + LEFT_Y_MIN) * 0.5) / (LEFT_Y_MAX - LEFT_Y_MIN) - LEFT_Y_ZERO, -1, 1);
    }

    private final double RIGHT_X_MIN=-1;
    private final double RIGHT_X_MAX = 1;
    private double RIGHT_X_ZERO = 0;
    public double getDriverRightX() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(4) - (RIGHT_X_MAX + RIGHT_X_MIN) * 0.5) / (RIGHT_X_MAX - RIGHT_X_MIN) - RIGHT_X_ZERO, -1, 1);
    }
    private final double RIGHT_Y_MIN = -1;
    private final double RIGHT_Y_MAX = 1;
    private double RIGHT_Y_ZERO = 0;
    public double getDriverRightY() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(5) - (RIGHT_Y_MAX + RIGHT_Y_MIN) * 0.5) / (RIGHT_Y_MAX - RIGHT_Y_MIN) - RIGHT_Y_ZERO, -1, 1);
    }

    //-------Driver Controller below

    /**
     * @return The Value of the driver controller's right Trigger
     * @param axis - The axis to read, starting at 0.
     * @return The value of the axis.
     */
    public double getDriverRightTrigger(){
        return driverController.getRawAxis(3);
    }

    /**
     * @return The Value of the driver controller's left Trigger
     * @param axis - The axis to read, starting at 0.
     * @return The value of the axis.
     */
    public double getDriverLeftTrigger(){
        return driverController.getRawAxis(2);
    }

    /**
     * @return The Value of the driver controller's left bumper
     * @param input - The current value of the input stream.
     * @return The debounced value of the input stream.     
     */
    public boolean getLeftBumper()
    {
        return parkingBrakeDebouncer.calculate(driverController.getRawButton(5));
    }

    /**
     * @return The Value of the driver controller's menu button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getRightBumper()
    {
        return driverController.getRawButton(6);
    }
    

    /**
     * @return The Value of the driver controller's 2 square button
     * @param button - The button index, beginning at 1.
     * @return Whether the button was pressed since the last check.
     */
    public boolean getFieldCentricToggle()
    {
        return fieldCentricDebouncer.calculate(driverController.getRawButton(7));
    }
    
    /**
     * @return The Value of the driver controller's menu button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getMenuButton()
    {
        return menuDriverButtonDebouncer.calculate(driverController.getRawButton(8));
    }
    
    /**
     * @return The Value of the driver controller's X button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getXButton()
    {
        return xDriverButtonDebouncer.calculate(driverController.getRawButton(3));
    }

    /**
     * @return The Value of the driver controller's A button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getAButton()
    {
        return aDriverButtonDebouncer.calculate(driverController.getRawButton(1));
    }

    /**
     * @return The Value of the driver controller's Y button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getYButton()
    {
        return yDriverButtonDebouncer.calculate(driverController.getRawButton(4));
    }

    /**
     * @return The Value of the driver controller's B button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getBButton()
    {
        return bDriverButtonDebouncer.calculate(driverController.getRawButton(2));
    }

    /**
     * @return The Value of the driver controller's DPad button
     * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
     */

    public int getDPad(){
        return driverController.getPOV();
    }

    public double getDriverTranslateX()
    {
        return getDriverLeftX();
    }

    public double getDriverTranslateY()
    {
        return getDriverLeftY();
    }

    public double getDriverRotate()
    {
        return getDriverRightX();
    }

    public String getDiagnostics() {
        return "";
    }

}