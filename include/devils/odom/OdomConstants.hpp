#include <cfloat>

namespace devils
{
    /**
     * Represents constant variables necessary for drive wheel, 2 deadwheel,
     * and 3 deadwheel odometry.
     */
    struct OdomConstants
    {
    public:
        /**
         * Constructor for a 2 wheel odometry system.
         * @param inchesPerRev the number of inches that corresponds to a 
         *      single revolution of a deadwheel; circumference of a deadwheel
         * @param parallelY Parallel deadwheel position offset relative to the
         *      robot's center of rotation. + further left, - further right
         * @param perpendicularX Perpendicular deadwheel position offset 
         *      relative to the robot's center of rotation. + further forward,
         *      - further backward
         * @param parallelReversed is the parallel deadwheel's rotation sensor
         *      direction reversed?
         * @param perpendicularReverse is the perpendicular deadwheel's
         *      rotation sensor direction reversed?
         */
        OdomConstants(
            double inchesPerTick,
            double parallelY,
            double perpendicularX,
            bool parallelReversed,
            bool perpendicularReverse
            )
        {
            INCHES_PER_REV = inchesPerTick;

            PARALLEL_A_Y    = parallelY;
            PERPENDICULAR_X = perpendicularX;

            PARALLEL_REVERSED      = parallelReversed;
            PERPENDICULAR_REVERSED = perpendicularReverse;

            // When new modes are implemented, set unused values to -DBL_MAX
        }

        // Getter for INCHES_PER_TICK
        double getInchesPerRev() const {
            return INCHES_PER_REV;
        }

        // Getter for PARALLEL_A_Y
        double getParallelAY() const {
            return PARALLEL_A_Y;
        }

        // Getter for PERPENDICULAR_X
        double getPerpendicularX() const {
            return PERPENDICULAR_X;
        }

        // Getter for PARALLEL_REVERSED
        bool isParallelReversed() const {
            return PARALLEL_REVERSED;
        }

        // Getter for PERPENDICULAR_REVERSED
        bool isPerpendicularReversed() const {
            return PERPENDICULAR_REVERSED;
        }

    private:
        // TODO: Implement constructor, values, and getters for 3 wheel odometry and drive wheel odometry
        double INCHES_PER_REV;

        // Deadwheel position offsets (relative to center of rotation)
        double PARALLEL_A_Y;      // + further left, - further right
        // name of Parallel Y adds extra identifier for 
        double PERPENDICULAR_X; // + further forward, - further back

        bool PARALLEL_REVERSED;
        bool PERPENDICULAR_REVERSED;
    };
}