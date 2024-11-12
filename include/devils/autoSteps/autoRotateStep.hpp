#pragma once
#include "pros/rtos.hpp"
#include "common/autoStep.hpp"
#include "devils/odom/odomSource.hpp"
#include "devils/chassis/chassisBase.hpp"
#include "devils/utils/math.hpp"

namespace devils
{
    /**
     * Represents a rotational step in an autonomous routine.
     */
    class AutoRotateStep : public IAutoStep
    {
    public:
        struct Options
        {
            /// @brief The distance to start accelerating in rads
            double accelDist = M_PI * 0.1;

            /// @brief The distance to start decelerating in rads
            double decelDist = M_PI * 0.4;

            /// @brief The maximum speed in %
            double maxSpeed = 0.4;

            /// @brief The minimum speed in %
            double minSpeed = 0.15;

            /// @brief The distance to the goal in radians
            double goalDist = 0.005;
        };

        /**
         * Creates a new rotational step.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param distance The distance to rotate in radians.
         */
        AutoRotateStep(ChassisBase &chassis, OdomSource &odomSource, double distance)
            : chassis(chassis),
              odomSource(odomSource),
              distance(distance)
        {
        }

        void doStep() override
        {
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();
            double startAngle = startPose.rotation;
            double targetAngle = startPose.rotation + distance;

            // Control Loop
            while (true)
            {
                // Calculate distance to start and target
                Pose currentPose = odomSource.getPose();
                double currentAngle = currentPose.rotation;
                double distanceToStart = startAngle - currentAngle;
                double distanceToTarget = targetAngle - currentAngle;

                // Calculate Speed
                double speed = Math::trapezoidProfile(
                    distanceToStart,
                    distanceToTarget,
                    options.accelDist,
                    options.decelDist,
                    options.minSpeed,
                    options.maxSpeed);

                // Debug
                NetworkTables::UpdateValue("StartAngle", startAngle);
                NetworkTables::UpdateValue("CurrentAngle", currentAngle);
                NetworkTables::UpdateValue("TargetAngle", targetAngle);
                NetworkTables::UpdateValue("AutoStepSpeed", speed);
                NetworkTables::UpdateValue("DistanceToTarget", distanceToTarget);
                NetworkTables::UpdateValue("DistanceToStart", distanceToStart);

                // Check if we are at the target
                if (fabs(distanceToTarget) < options.goalDist)
                    break;

                // Move Chassis
                chassis.move(0, speed);

                // Delay
                pros::delay(10);
            }

            // Stop Chassis
            chassis.stop();
        }

        Options &getOptions()
        {
            return options;
        }

    protected:
        // Options
        Options options = Options();

        // Robot Base
        ChassisBase &chassis;
        OdomSource &odomSource;

        // Drive Step Variables
        double distance = 0;
    };
}