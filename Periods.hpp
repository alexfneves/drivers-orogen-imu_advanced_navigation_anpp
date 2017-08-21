#ifndef IMU_ADVANCED_NAVIGATION_ANPP_OROGEN_PERIODS_HPP
#define IMU_ADVANCED_NAVIGATION_ANPP_OROGEN_PERIODS_HPP

#include <base/Time.hpp>

namespace imu_advanced_navigation_anpp
{
    /** Configuration of the update periods
     *
     * All periods are expressed in multiple of the device sampling period,
     * which is configured through the configuration property. The default
     * sampling period is 1ms
     *
     * Set a period to zero to disable
     */
    struct Periods
    {
        /** How often the world-relative RBS will be updated
         *
         * The default is 10
         */
        int world_pose;

        /** How often the body-relative RBS will be updated
         *
         * The default is 10
         */
        int body_velocity;

        /** How often the acceleration will be updated
         *
         * The default is 1
         */
        int acceleration;

        /** How often the raw sensors will be updated
         * 
         * The default is 1
         */
        int imu_sensors;

        /** How often the GNSS info will be updated
         *
         * The default is zero (disabled)
         */
        int gnss_info;

        /** How often the GNSS info will be updated
         *
         * The default is zero (disabled)
         */
        int gnss_satellite_info;

        /** How often the north seeking initialization status will be reported
         * 
         * The default is 1000
         */
        int north_seeking;

        Periods()
            : world_pose(10)
            , body_velocity(10)
            , acceleration(1)
            , imu_sensors(1)
            , gnss_info(0)
            , gnss_satellite_info(0)
            , north_seeking(1000) {}
    };
}

#endif

