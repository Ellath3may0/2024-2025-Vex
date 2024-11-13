#pragma once
#include "../hardware/imu.hpp"
#include "../hardware/rotationSensor.hpp"
#include "../geometry/pose.hpp"
#include "odomSource.hpp"
#include "../utils/runnable.hpp"
#include "../odom/OdomConstants.hpp"
#include <cmath>

#define M_PI 3.14159265358979323846

namespace devils
{
    class DeadWheelIMUOdom : public OdomSource, public Runnable
    {
    public:
        
        DeadWheelIMUOdom(
            int8_t paraPort,
            int8_t perpPort,
            int8_t imuPort,
            Pose p,
            OdomConstants odomConstants
            )
        {
            // Starting pose and hardware
            pose = Pose(p.x, p.y, p.rotation);
            parallelDeadwheel       = new RotationSensor("Parallel Deadwheel", paraPort);
            perpendicularDeadwheel  = new RotationSensor("Perpendicular Deadwheel", perpPort);
            imu                     = new IMU("IMU", imuPort);

            // Constants
            INCHES_PER_REV = odomConstants.getInchesPerRev();

            PARALLEL_Y      = odomConstants.getParallelAY();
            PERPENDICULAR_X = odomConstants.getPerpendicularX();

            PARALLEL_REVERSED      = odomConstants.isParallelReversed();
            PERPENDICULAR_REVERSED = odomConstants.isPerpendicularReversed();

            lastPara = parallelDeadwheel->getAngle() / (2 * M_PI);
            lastPerp = perpendicularDeadwheel->getAngle() / (2 * M_PI);
            lastHeading = imu->getHeading();
        }

        void updatePose()
        {
            // New reads
            int currentTime = pros::millis();
            double currentPara = parallelDeadwheel->getAngle() / (2 * M_PI);
            double currentPerp = perpendicularDeadwheel->getAngle() / (2 * M_PI);
            double currentHeading = imu->getHeading();

            // Change since last udpate
            int deltaTime = currentTime - lastTime;
            double deltaPara = currentPara - lastPara;
            double deltaPerp = currentPerp - lastPerp;
            double deltaHeading = currentHeading - lastHeading;

            // Complicated math that I stole from roadrunner
            double u;
            if (deltaHeading >= 0)
            {
                u = deltaHeading + DBL_EPSILON;
            }
            else
            {
                u = deltaHeading - DBL_EPSILON;
            }

            // Calculate translationVector
            double vectorX = deltaPara * INCHES_PER_REV - PARALLEL_Y * deltaHeading;
            double vectorY = deltaPerp * INCHES_PER_REV - PERPENDICULAR_X * deltaHeading;

            // Calculate rotated vector
            double c = 1 - cos(u);
            double s = sin(u);
            double translationX = (s * vectorX - c * vectorY) / u;
            double translationY = (c * vectorX + s * vectorY) / u;

            // Update pose
            pose.x += translationX;
            pose.y += translationY;
            pose.rotation = currentHeading;
        }

        void setPose(Pose &p)
        {
            pose = p;
        }

        /**
         * Gets the current pose of the robot
         * @return Pose position and heading of the robot
         */
        Pose &getPose()
        {
            return pose;
        }

    private:

        // Hardware
        RotationSensor* parallelDeadwheel = nullptr;
        RotationSensor* perpendicularDeadwheel = nullptr;
        IMU*            imu;

        // All measurements are in inches and all relative directions are 
        // as if you were looking at the robot from above, with its front
        // facing away from you.
        /*
                   FRONT (Y+)
                _______________
                |             |
                |             |
        (x+)LEFT|     TOP     |RIGHT (X-)
                |             |
                |             |
                |_____________|
                   BACK (Y-)
        */


        // Constants
        double INCHES_PER_REV;

        // Deadwheel position offsets (relative to center of rotation)
        double PARALLEL_Y;      // + further left, - further right
        double PERPENDICULAR_X; // + further forward, - further back

        bool PARALLEL_REVERSED;
        bool PERPENDICULAR_REVERSED;

        
        // Variables
        Pose pose;

        uint32_t lastTime = 0;
        double lastPara;
        double lastPerp;
        double lastHeading; // This is in radians (FIGHT ME ITS BETTER)

        // Utility methods
        int signum(int i)
        {
            if (i > 0)
            {
                return 1;
            }
            else if (i < 0)
            {
                return -1;
            }
            else
            {
                return 0;
            }
        }
    };
}