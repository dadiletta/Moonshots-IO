/* Created Tue Jan 24 09:27:35 EST 2017 */
package org.frc5973.robot;

import java.text.DecimalFormat;
import java.util.concurrent.TimeUnit;

import org.strongback.Strongback;
import org.strongback.SwitchReactor;
import org.strongback.components.Motor;
import org.strongback.components.ui.ContinuousRange;
import org.strongback.components.ui.FlightStick;
import org.strongback.drive.TankDrive;
import org.strongback.hardware.Hardware;
import org.strongback.util.Values;

import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot {
	// commentpublic class Robot extends IterativeRobot {

	// motor declarations
	private static final int JOYSTICK_PORT = 0; // in driver station
	private static final int RMOTOR_FRONT = 3;
	private static final int RMOTOR_REAR = 2;
	private static final int LMOTOR_FRONT = 0;
	private static final int LMOTOR_REAR = 1;

	private static final boolean SLOW_ON_AUTONOMOUS = false;
	private TankDrive drive;
	private ContinuousRange driveSpeed;
	private ContinuousRange turnSpeed;

	// We moved this up here so we can output this variable in the teleop
	protected ContinuousRange sensitivity;
	// Used to limit and format the number of console outputs
	private int filter = 0;
	private String pattern = "###.###";
	private DecimalFormat myFormat = new DecimalFormat(pattern);
	private double sen;

	@Override
	public void robotInit() {// Execution period extended so it doesn't time out
								// per cycle
		// Records no data and no events, a feature of Strongback we will look
		// at next year
		Strongback.configure().recordNoData().recordNoCommands().recordNoEvents()
				.useExecutionPeriod(200, TimeUnit.MILLISECONDS).initialize();

		// Set up the robot hardware ...
		Motor left_front = Hardware.Motors.victorSP(LMOTOR_FRONT).invert(); // left
																			// rear
		Motor left_rear = Hardware.Motors.victorSP(LMOTOR_REAR).invert(); // left
																			// front
		// DoubleToDoubleFunction SPEED_LIMITER = Values.limiter(-0.1, 0.1);
		Motor right_front = Hardware.Motors.victorSP(RMOTOR_FRONT); // right
																	// rear
		Motor right_rear = Hardware.Motors.victorSP(RMOTOR_REAR); // right front

		Motor left = Motor.compose(left_front, left_rear);
		Motor right = Motor.compose(right_front, right_rear);
		drive = new TankDrive(left, right);
		// Set up the human input controls for teleoperated mode. We want to use
		// the Logitech Attack 3D's throttle as a
		// "sensitivity" input to scale the drive speed and throttle, so we'll
		// map it from it's native [-1,1] to a simple scale
		// factor of [0,1] ...
		FlightStick joystick = Hardware.HumanInterfaceDevices.logitechExtreme3D(JOYSTICK_PORT);
		SwitchReactor reactor = Strongback.switchReactor();
		ContinuousRange sensitivity = joystick.getThrottle().map(t -> ((t + 1.0) / 2.0));
		sensitivity = joystick.getThrottle().map(Values.mapRange(-1.0, 1.0).toRange(0.0, 1.0));
		driveSpeed = joystick.getPitch().scale(sensitivity::read); // scaled
		turnSpeed = joystick.getRoll().scale(sensitivity::read); // scaled and
																	// inverted
	}

	@Override
	public void teleopInit() {
		// Kill anything running if it is ...
		Strongback.disable();
		// Start Strongback functions ...
		Strongback.start();
	}

	@Override
	public void teleopPeriodic() {
		drive.arcade(driveSpeed.read(), turnSpeed.read());

	}

	@Override
	public void disabledInit() {
		// Tell Strongback that the robot is disabled so it can flush and kill
		// commands.
		Strongback.disable();
	}

}
