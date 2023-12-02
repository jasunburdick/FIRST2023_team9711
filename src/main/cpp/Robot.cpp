// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
// #include "commands/DriveDistance.cpp"
#include "commands/DriveDistance.h"
#include "subsystems/Drivetrain.h"
//________________________________________________________________________________________________________

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/length.h>

void Robot::RobotInit() {
  // Add the ultrasonic on the "Sensors" tab of the dashboard
  // Data will update automatically
  frc::Shuffleboard::GetTab("Sensors").Add(m_rangeFinder);
}

void Robot::TeleopPeriodic() {
  // We can read the distance
  units::meter_t distance = m_rangeFinder.GetRange();
  // units auto-convert
  units::millimeter_t distanceMillimeters = distance;
  units::inch_t distanceInches = distance;

  // We can also publish the data itself periodically
  frc::SmartDashboard::PutNumber("Distance[mm]", distanceMillimeters.value());
  frc::SmartDashboard::PutNumber("Distance[inch]", distanceInches.value());
}

void Robot::TestInit() {
  // By default, the Ultrasonic class polls all ultrasonic sensors in a
  // round-robin to prevent them from interfering from one another. However,
  // manual polling is also possible -- note that this disables automatic mode!
  m_rangeFinder.Ping();
}

void Robot::TestPeriodic() {
  if (m_rangeFinder.IsRangeValid()) {
    // Data is valid, publish it
    units::millimeter_t distanceMillimeters = m_rangeFinder.GetRange();
    units::inch_t distanceInches = m_rangeFinder.GetRange();
    frc::SmartDashboard::PutNumber("Distance[mm]", distanceMillimeters.value());
    frc::SmartDashboard::PutNumber("Distance[inch]", distanceInches.value());

    // Ping for next measurement
    m_rangeFinder.Ping();
  }
}

void Robot::TestExit() {
  // Enable automatic mode
  frc::Ultrasonic::SetAutomaticMode(true);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif







//_____________________________________________________________________________________________________

// //This function executes when the robot is first enabled
// void Robot::RobotInit() {}
// /**
//  * This function is called every 20 ms, no matter the mode. Use
//  * this for items like diagnostics that you want to run during disabled,
//  * autonomous, teleoperated and test.
//  *
//  * <p> This runs after the mode specific periodic functions, but before
//  * LiveWindow and SmartDashboard integrated updating.
//  */
// void Robot::RobotPeriodic() {
//   frc2::CommandScheduler::GetInstance().Run();
// }

// /**
//  * This function is called once each time the robot enters Disabled mode. You
//  * can use it to reset any subsystem information you want to clear when the
//  * robot is disabled.
//  */
// void Robot::DisabledInit() {}

// void Robot::DisabledPeriodic() {}

// /**
//  * This autonomous runs the autonomous command selected by your {@link
//  * RobotContainer} class.
//  */
// void Robot::AutonomousInit() {
//   // m_autonomousCommand = m_container.GetAutonomousCommand();

//   // if (m_autonomousCommand != nullptr) {
//   //   m_autonomousCommand->Schedule();
//   // }
//   // Drivetrain* drive;
  
// }

// void Robot::AutonomousPeriodic() {}

// void Robot::TeleopInit() {
//   // This makes sure that the autonomous stops running when
//   // teleop starts running. If you want the autonomous to
//   // continue until interrupted by another command, remove
//   // this line or comment it out.
//   if (m_autonomousCommand != nullptr) {
//     m_autonomousCommand->Cancel();
//     m_autonomousCommand = nullptr;
//   }
// }

// /**
//  * This function is called periodically during operator control.
//  */
// void Robot::TeleopPeriodic() {}

// /**
//  * This function is called periodically during test mode.
//  */
// void Robot::TestPeriodic() {}

// #ifndef RUNNING_FRC_TESTS
// int main() {
//   return frc::StartRobot<Robot>();
// }
// #endif
