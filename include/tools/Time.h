#pragma once

#include <string>
#include <vector>
#include <map>
#include <chrono>

namespace amp {

enum class TimeUnit {
    us, // microseconds
    ms, // milliseconds
    s   // seconds
};

class Profiler {
    public:
        /// @brief Get the time clocked by the most recent timer. This is reset every time a new timer is created under the given key
        /// @param key Key to get the profile of
        /// @param unit Time unit
        /// @return Time in the unit provided 
        static double getMostRecentProfile(const std::string& key, TimeUnit unit = TimeUnit::ms);

        /// @brief Get the total (cumulative) amount of time spent running timers under a given key.
        /// @param key Key to get the profile of
        /// @param unit Time unit
        /// @return Time in the unit provided 
        static double getTotalProfile(const std::string& key, TimeUnit unit = TimeUnit::ms);

    private:
        struct Data {
            std::chrono::microseconds most_recent_duration;
            std::chrono::microseconds total_duration;
        };

    private:
        static Data& getData(const std::string& key);
    private:
        inline static std::map<std::string, Data> s_profiles;
        friend class Timer;
};

/// @brief Construct an object wherever you want to begin timing. 
class Timer {
    public:
        /// @brief Pass a key that can be looked up with the profiler
        /// @param key Key related to this timer
        Timer(const std::string& key);
        
        /// @brief Manually ends the timer before the object is destroyed
        void stop();

        /// @brief Get the current reading of the timer
        /// @param unit Time unit
        /// @return Time in the unit provided
        double now(TimeUnit unit = TimeUnit::ms);
        
        /// @brief Automatically ends the timer when the object is destroyed
        ~Timer();
    public:
        std::string m_key;
        std::chrono::system_clock::time_point m_start;
};

}