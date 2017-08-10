#ifndef IMU_ADVANCED_NAVIGATION_ANPP_OROGEN_PERIODS_HPP
#define IMU_ADVANCED_NAVIGATION_ANPP_OROGEN_PERIODS_HPP

#include <base/Time.hpp>

namespace imu_advanced_navigation_anpp
{
    /** Configuration of the update periods
     */
    struct Periods
    {
        /** The base packet period
         *
         * All periods below are expressed in multiples of this value
         */
        base::Time base;

        /** Whether timestamping should use the device-provided time
         *
         * Enable this only if the device is connected to a GPS
         */
        bool use_device_time;

        /** How often the world-relative RBS will be updated
         *
         * Set to zero to disable
         *
         * Expressed in multiples of the base period
         */
        int world_pose;

        /** How often the body-relative RBS will be updated
         *
         * Set to zero to disable
         *
         * Expressed in multiples of the base period
         */
        int body_pose;

        /** How often the acceleration will be updated
         *
         * Set to zero to disable
         *
         * Expressed in multiples of the base period
         */
        int acceleration;

        /** How often the raw sensors will be updated
         *
         * Set to zero to disable
         *
         * Expressed in multiples of the base period
         */
        int raw_sensors;

        /** How often the GNSS info will be updated
         *
         * Set to zero to disable
         *
         * Expressed in multiples of the base period
         */
        int gnss_info;

        /** How often the GNSS info will be updated
         *
         * Set to zero to disable
         *
         * Expressed in multiples of the base period
         */
        int gnss_satellites;

        Periods()
            : base(base::Time::fromMilliseconds(10))
            , use_device_time(false)
            , world_pose(10)
            , body_pose(10)
            , acceleration(1)
            , raw_sensors(1)
            , gnss_info(0)
            , gnss_satellites(0) {}
    };
}

#endif

