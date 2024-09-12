
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  // public LaserCan lc; 
  // public LaserCan lc2;

  // @Override
  // public void robotInit() {
  //   lc = new LaserCan(0);
  //   lc2 = new LaserCan(1);
  //   // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
  //   try {
  //     lc.setRangingMode(LaserCan.RangingMode.SHORT);
  //     lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
  //     lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
  //   } catch (ConfigurationFailedException e) {
  //     System.out.println("Configuration failed! " + e);
  //   }

  //   try {
  //     lc2.setRangingMode(LaserCan.RangingMode.SHORT);
  //     lc2.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
  //     lc2.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
  //   } catch (ConfigurationFailedException e) {
  //     System.out.println("Configuration failed! " + e);
  //   }
  // }

  // @Override
  // public void robotPeriodic() {
  //   LaserCan.Measurement measurement = lc.getMeasurement();
  //   if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
  //     System.out.println("The target is " + measurement.distance_mm + "mm away!");
  //   } else {
  //     System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
  //     // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
  //   }

  //   LaserCan.Measurement measurement2 = lc2.getMeasurement();
  //   if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
  //     System.out.println("The target is " + measurement2.distance_mm + "mm away!");
  //   } else {
  //     System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
  //     // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
  //   }
  //}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
