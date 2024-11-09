#pragma once
#include "pros/rtos.hpp"

namespace devils
{
    class INetworkObject;
    typedef std::vector<devils::INetworkObject *> NetworkObjectList;

    /**
     * Represents a netowrk object that can be serialized and sent over serial.
     */
    class INetworkObject
    {
    public:
        // Disable copy constructor
        INetworkObject(INetworkObject const &) = delete;

        // Disable assignment operator
        INetworkObject &operator=(INetworkObject const &) = delete;

        // Register the network object
        INetworkObject()
        {
            GetAllNetworkObjects().push_back(this);
        }

        // Unregister the network object
        ~INetworkObject()
        {
            NetworkObjectList &allNetworkObjects = GetAllNetworkObjects();

            allNetworkObjects.erase(
                std::remove(allNetworkObjects.begin(), allNetworkObjects.end(), this),
                allNetworkObjects.end());
        }

        /**
         * Gets a list of all alive network objects.
         * @return A list of all alive network objects.
         */
        static NetworkObjectList &GetAllNetworkObjects()
        {
            static NetworkObjectList allNetworkObjects;
            return allNetworkObjects;
        }

        /**
         * Sets the rate at which the object should be serialized.
         * Increasing the rate will decrease the frequency of serialization for lower priority objects.
         * @param maxSerializeTime The rate at which the object should be serialized in milliseconds.
         */
        void setSerializationRate(int maxSerializeTime)
        {
            this->maxSerializeTime = maxSerializeTime;
        }

        /**
         * Checks if the object is dirty
         * @return True if the object is dirty, false otherwise.
         */
        bool isDirty()
        {
            return pros::millis() - lastSerializationTime > maxSerializeTime;
        }

        /**
         * Runs the serialization of the object.
         * Updates the last serialization time to clean the object.
         * Usually called automatically in `NetworkService::update`.
         */
        void runSerialization()
        {
            serialize();
            lastSerializationTime = pros::millis();
        }

    protected:
        /**
         * Should be implemented by child classes to serialize the object.
         * Child classes can call the `NetworkTable::UpdateValue` method to update the data.
         */
        virtual void serialize() = 0;

    private:
        int maxSerializeTime = 50; // ms
        int lastSerializationTime = -1;
    };
}