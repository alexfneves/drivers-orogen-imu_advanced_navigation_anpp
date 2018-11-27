/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <imu_advanced_navigation_anpp/Driver.hpp>

using namespace imu_advanced_navigation_anpp;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    Driver* driver = new Driver();
    if (!_io_port.get().empty())
        driver->openURI(_io_port.get());
    setDriver(driver);
    mDriver = driver;

    if (! TaskBase::configureHook())
        return false;

    driver->setConfiguration(_configuration.get());
    driver->setUseDeviceTime(_use_device_time.get());

    driver->setStatusPeriod(1);

    mPeriods = _periods.get();
    driver->setPositionPeriod(mPeriods.world_pose);
    driver->setOrientationPeriod(mPeriods.world_pose);
    driver->setNEDVelocityPeriod(mPeriods.world_pose);
    driver->setAccelerationPeriod(mPeriods.acceleration);
    driver->setBodyVelocityPeriod(mPeriods.body_velocity);
    driver->setAngularVelocityPeriod(mPeriods.body_velocity);
    driver->setAngularAccelerationPeriod(mPeriods.acceleration);
    driver->setRawSensorsPeriod(mPeriods.imu_sensors);
    driver->setGNSSPeriod(mPeriods.gnss_info);
    driver->setGNSSSatelliteSummaryPeriod(mPeriods.gnss_info);
    driver->setGNSSSatelliteDetailsPeriod(mPeriods.gnss_satellite_info);
    driver->setNorthSeekingInitializationStatusPeriod(mPeriods.north_seeking);

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    mDriver->clear();
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
}

void Task::processIO()
{
    int period = mDriver->poll();
    if (period <= 0)
        return;

    if (period == 1)
    {
        mStatus = mDriver->getIMUStatus();
        _status.write(mStatus);
    }

    if (period == mPeriods.world_pose)
    {
        base::samples::RigidBodyState rbs = mDriver->getWorldRigidBodyState();
        rbs.sourceFrame = _imu_frame.get();
        rbs.targetFrame = _nwu_frame.get();
        if (!mStatus.isNavigationInitialized())
        {
            rbs.invalidatePosition();
            rbs.invalidatePositionCovariance();
            rbs.invalidateVelocity();
            rbs.cov_orientation(2, 2) = (M_PI * M_PI);
        }
        if (mStatus.isOrientationInitialized())
            _nwu_pose_samples.write(rbs);
    }
    if (period == mPeriods.body_velocity)
    {
        base::samples::RigidBodyState rbs = mDriver->getBodyRigidBodyState();
        rbs.sourceFrame = _imu_frame.get();
        rbs.targetFrame = _imu_frame.get();
        if (!mStatus.isNavigationInitialized())
        {
            rbs.invalidatePosition();
            rbs.invalidateVelocity();
        }
        if (mStatus.isOrientationInitialized())
            _body_velocity_samples.write(rbs);
    }
    if (mStatus.isNavigationInitialized() && period == mPeriods.acceleration)
    {
        base::samples::RigidBodyAcceleration accel = mDriver->getAcceleration();
        _acceleration_samples.write(accel);
    }
    if (period == mPeriods.imu_sensors)
    {
        base::samples::IMUSensors sample = mDriver->getIMUSensors();
        _imu_sensors_samples.write(sample);
    }
    if (period == mPeriods.gnss_info)
    {
        gps_base::Solution solution = mDriver->getGNSSSolution();
        gps_base::SolutionQuality quality = mDriver->getGNSSSolutionQuality();
        _gps_solution_samples.write(solution);
        _gps_solution_quality_samples.write(quality);
    }
    if (period == mPeriods.gnss_satellite_info)
    {
        gps_base::SatelliteInfo sample = mDriver->getGNSSSatelliteInfo();
        _gps_satellite_info_samples.write(sample);
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    setDriver(nullptr);
    delete mDriver;
    mDriver = nullptr;
}
