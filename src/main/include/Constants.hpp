#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>
const double kJoystickDeadband = 0.05;

/**
 * Whether we're building for the 2021 Infinite Recharge At-Home Challenge
 * (IR\@H).
 *
 * This enables IR\@H-specific autonomous modes and flywheel speed presets.
 */
const bool kAtHomeChallenge = false;

/// The period at which feedback controllers run
const units::second_t kControllerPeriod = 5_ms;

const double kP = 0.15;
const double kI = 0;
const double kD = 0.0;


const units::voltage::volt_t kS {0.1695};
const units::unit_t<units::compound_unit<units::voltage::volts, units::inverse<units::velocity::meters_per_second>>> kV {2.8559};
const units::unit_t<units::compound_unit<units::voltage::volts, units::inverse<units::compound_unit<units::velocity::meters_per_second, units::inverse<units::time::seconds>>>>> kA{0.4864};
const units::time::second_t dt{0.02};