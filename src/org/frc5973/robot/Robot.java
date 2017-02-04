/* Created Tue Jan 24 09:27:35 EST 2017 */
package org.frc5973.robot;

import org.strongback.Strongback;

import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot {

    @Override
    public void robotInit() {
    }

    @Override
    public void teleopInit() {
        // Start Strongback functions ...
        Strongback.start();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
        // Tell Strongback that the robot is disabled so it can flush and kill commands.
        Strongback.disable();
    }

}
