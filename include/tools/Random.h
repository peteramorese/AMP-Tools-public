#pragma once

#include <random>

#include "tools/Logging.h"

namespace amp {

class RNG {
    public:
        /// @brief Universal Unique Identifier (64bit width)
        /// @return uuid
        inline static uint64_t uuid64() {
            return s_int_dist()(s_random_gen_64());
        }

        /// @brief Random 32bit integer
        inline static int randiUnbounded() {
            return s_int_dist()(s_random_gen());
        }

        /// @brief Seeded random 32 bit integer
        inline static int srandiUnbounded() {
            return s_int_dist()(s_seeded_gen());
        }

        /// @brief Random 32bit integer between [lower, upper)
        /// @param lower Lower bound
        /// @param upper Upper bound
        inline static int randi(int lower, int upper) {
            ASSERT(lower <= upper, "Upper bound must be geq to lower bound");
            int diff = (upper - lower);
            return ((randiUnbounded() % diff) + diff) % diff + lower;
        }

        /// @brief Seeded random 32bit integer between [lower, upper)
        /// @param lower Lower bound
        /// @param upper Upper bound
        inline static int srandi(uint32_t lower, uint32_t upper) {
            ASSERT(lower <= upper, "Upper bound must be geq to lower bound");
            int diff = (upper - lower);
            return ((srandiUnbounded() % diff) + diff) % diff + lower;
        }

        /// @brief Random float between [lower, upper]
        /// @param lower Lower bound
        /// @param upper Upper bound
        inline static float randf(float lower = 0.0f, float upper = 1.0f) {
            ASSERT(lower <= upper, "Upper bound must be geq to lower bound");
            return (upper - lower) * s_real_dist_f()(s_random_gen()) + lower;
        }

        /// @brief Seeded float double between [lower, upper]
        /// @param lower Lower bound
        /// @param upper Upper bound
        inline static float srandf(float lower = 0.0f, float upper = 1.0f) {
            ASSERT(lower <= upper, "Upper bound must be geq to lower bound");
            return (upper - lower) * s_real_dist_f()(s_seeded_gen()) + lower;
        }

        /// @brief Random double between [lower, upper]
        /// @param lower Lower bound
        /// @param upper Upper bound
        inline static float randd(double lower = 0.0, double upper = 1.0) {
            ASSERT(lower <= upper, "Upper bound must be geq to lower bound");
            return (upper - lower) * s_real_dist_d()(s_random_gen()) + lower;
        }

        /// @brief Seeded random double between [lower, upper]
        /// @param lower Lower bound
        /// @param upper Upper bound
        inline static float srandd(double lower = 0.0, double upper = 1.0) {
            ASSERT(lower <= upper, "Upper bound must be geq to lower bound");
            return (upper - lower) * s_real_dist_d()(s_seeded_gen()) + lower;
        }

        /// @brief Seed the srandom number generator to re-seed all srand-methods
        /// @param s Seed
        inline static void seed(uint32_t s) {s_seeded_gen().seed(s);}

        /// @brief Normally distributed random number (mean = 0.0f, std = 1.0)
        inline static float nrand() {
            std::normal_distribution<> dist;
            return dist(s_random_gen());
        }

        /// @brief Normally distributed random number
        /// @param mean Mean
        /// @param std Standard deviation
        inline static float nrand(double mean, double std) {
            std::normal_distribution<> dist(mean, std);
            return dist(s_random_gen());
        }

        /// @brief Seeded normally distributed random number (mean = 0.0f, std = 1.0)
        inline static float nsrand() {
            std::normal_distribution<> dist;
            return dist(s_seeded_gen());
        }

        /// @brief Seeded normally distributed random number
        /// @param mean Mean
        /// @param std Standard deviation
        inline static float nsrand(double mean, double std) {
            std::normal_distribution<> dist(mean, std);
            return dist(s_seeded_gen());
        }
    private:
        inline static std::random_device& s_rd() {static std::random_device rd; return rd;}
        inline static std::mt19937& s_random_gen() {static thread_local std::mt19937 gen(s_rd()()); return gen;}
        inline static std::mt19937_64& s_random_gen_64() {static thread_local std::mt19937_64 gen(s_rd()()); return gen;}
        inline static std::mt19937& s_seeded_gen() {static thread_local std::mt19937 gen(123); return gen;}
        inline static std::uniform_real_distribution<float>& s_real_dist_f() {static std::uniform_real_distribution<float> real_dist(0.0f, 1.0f); return real_dist;}
        inline static std::uniform_real_distribution<double>& s_real_dist_d() {static std::uniform_real_distribution<double> real_dist(0.0, 1.0); return real_dist;}
        inline static std::uniform_int_distribution<>& s_int_dist() {static std::uniform_int_distribution<> int_dist(INT32_MIN, INT32_MAX); return int_dist;}

};

}