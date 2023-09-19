#pragma once

#include <string>
#include <vector>
#include <exception>
#include <set>

#include "tools/Logging.h"

namespace amp {

    template <typename T>
    class DataArgument {
        public:
            void set(const T& arg) {m_arg = arg;}
            void set(T&& arg) {m_arg = std::move(arg);}
            const T& get() const {
                return m_arg;
            }
        private:
            T m_arg;
    };

    class IndicatorArgument {};

    template <typename T, typename BASE = typename std::conditional<!std::is_same<T, void>::value, DataArgument<T>, IndicatorArgument>::type>
    class Argument : public BASE {
        public:
            Argument(bool valid) : m_valid(valid) {}
            inline bool has() const {return m_valid;}
        private:
            bool m_valid;
    };

    template <typename T>
    class ArrayArgument {
        public:
            ArrayArgument(const std::vector<T>& args, bool valid) : m_args(args), m_valid(valid) {}
            const std::vector<T>& get() const {return m_args;}
            operator bool() const {return m_valid;}
        private:
            std::vector<T> m_args;
            bool m_valid;
    };

    class ArgParser {
        public:
            ArgParser(int argc, char** argv) 
                : m_argc(argc)
                , m_argv(argv)
                , m_checked(argc, false) 
            {
                for (int i = 1; i < argc; ++i) {
                    std::string arg = argv[i];
                    if (arg == "--help" || arg == "-h") 
                        m_help = true;
                }
                m_unique_keys.insert("help");
                m_unique_flags.insert('h');
            }

            // Parse the argument given a key, flag, default value, and description. Note that using type T = char
            // may not use a character flag without default value
            template <typename T>
            Argument<T> parse(const std::string& key, char flag, const T& default_value, const std::string& description) {
                static_assert(!std::is_same<T, void>::value, "Do not specify default value with indicator");
                return _parse<T>(&flag, &key, &default_value, description);
            }

            template <typename T>
            Argument<T> parse(const std::string& key, const T& default_value, const std::string& description) {
                static_assert(!std::is_same<T, void>::value, "Do not specify default value with indicator");
                return _parse<T>(nullptr, &key, &default_value, description);
            }

            template <typename T>
            Argument<T> parse(char flag, const T& default_value, const std::string& description) {
                static_assert(!std::is_same<T, void>::value, "Do not specify default value with indicator");
                return _parse<T>(&flag, nullptr, &default_value, description);
            }

            template <typename T = void, typename _ENABLE = typename std::enable_if<!std::is_same<T, char>::value>::type>
            Argument<T> parse(const std::string& key, char flag, const std::string& description) {
                //static_assert(!std::is_same)
                return _parse<T>(&flag, &key, nullptr, description);
            }

            template <typename T = void>
            Argument<T> parse(const std::string& key, const std::string& description) {
                return _parse<T>(nullptr, &key, nullptr, description);
            }

            template <typename T = void>
            Argument<T> parse(char flag, const std::string& description) {
                return _parse<T>(&flag, nullptr, nullptr, description);
            }

            // Include this to enable the help function and check for unrecognized arguments
            bool enableHelp() {
                if (m_help) {
                    PRINT("\n   [Help]\n");

                    std::vector<std::string> key_and_flag_strs;
                    key_and_flag_strs.reserve(m_descriptions.size());

                    std::size_t max_length = 0;
                    for (auto&[flag, key, _, __] : m_descriptions) {
                        std::string key_and_flag;
                        if (!flag.empty() && !key.empty()) {
                            key_and_flag = key + " or " + flag;
                        } else if (!key.empty()) {
                            key_and_flag = key;
                        } else if (!flag.empty()) {
                            key_and_flag = flag;
                        } else {
                            ASSERT(false, "Empty key and flag");
                        }

                        if (key_and_flag.size() > max_length)
                            max_length = key_and_flag.size();

                        key_and_flag_strs.push_back(std::move(key_and_flag));
                    }

                    std::string help_key_and_flag = "--help or -h";
                    help_key_and_flag += std::string(max_length - help_key_and_flag.size() + 1, ' ');
                    PRINT_NAMED(help_key_and_flag, "Display this message");

                    std::size_t ind = 0;
                    for (auto&[flag, key, def, desc] : m_descriptions) {
                        std::string description = desc + " ";

                        if (!def.empty()) description += " [Default value: " + def + "]";
                        std::string key_and_flag_str_adj = key_and_flag_strs[ind++];
                        key_and_flag_str_adj += std::string(max_length - key_and_flag_str_adj.size() + 1, ' ');
                        PRINT_NAMED(key_and_flag_str_adj, description);
                    }

                    NEW_LINE;

                    std::exit(0);
                    return false;
                }

                for (uint32_t i=1; i<m_argc; ++i) {
                    if (!m_checked[i]) {
                        ERROR("Unrecognized arg '" << m_argv[i] << "'");
                        std::exit(1);
                        return false;
                    }
                }
                return true;
            }

        private:
            struct Documentation {
                Documentation(const std::string& flag_, const std::string& key_, const std::string& default_value_, const std::string& description_)
                    : flag(flag_)
                    , key(key_)
                    , default_value(default_value_)
                    , description(description_)
                {}

                std::string flag;
                std::string key;
                std::string default_value;
                std::string description;
            };
        private:
            inline std::string getFlag(char flag) const {
                return "-" + std::string{flag};
            }
            inline std::string getKey(const std::string& key) const {return "--" + key;}

            template <typename T>
            T to(const std::string& str);

            template <typename T>
            std::string from(const T& v);

            bool isValue(const std::string& arg) const {
                return !arg.empty() && arg[0] != '-';
            }
        
            template <typename T>
            Argument<T> _parse(const char* flag, const std::string* key, const T* default_value, const std::string& description) {
                // Determines if the argument is just an indicator, or holding a value in the next slot
                constexpr bool indicator = std::is_same<T, void>::value;

                if (!key && !flag) {
                    throw std::invalid_argument("Both key and flag cannot be unspecified");
                }

                if (key && (m_unique_keys.find(*key) != m_unique_keys.end())) {
                    ASSERT(false, "Duplicate key: " << *key);
                } else if (key) {
                    m_unique_keys.insert(*key);
                }

                if (flag && (m_unique_flags.find(*flag) != m_unique_flags.end())) {
                    ASSERT(false, "Duplicate flag: " << *flag);
                } else if (flag) {
                    m_unique_flags.insert(*flag);
                }

                // Flag
                std::string flag_str = std::string();
                if (flag) {
                    flag_str = getFlag(*flag);
                }

                // Key
                std::string key_str = std::string();
                if (key) {
                    ASSERT(isValue(*key), "Do not use dashes '--' when specifying a key label");
                    key_str = getKey(*key);
                }

                // Default
                std::string default_str = std::string();
                if constexpr (!indicator) {
                    if (default_value)
                        default_str = from<T>(*default_value);
                }

                // Add the necessary descriptions
                m_descriptions.emplace_back(flag_str, key_str, default_str, description);

                if (m_help)
                    return Argument<T>(true);

                for (uint32_t i=1; i<m_argc; ++i) {
                    if (m_checked[i]) continue;

                    std::string arg = m_argv[i];
                    if ((flag && arg == flag_str) || (key && arg == key_str)) {
                        m_checked[i] = true;
                        if constexpr (indicator) {
                            return Argument<T>(true);
                        } else {
                            ASSERT(i < m_argc - 1 && isValue(m_argv[i + 1]), "Parse key: '" << key_str << "' expected a value (use type T = void for indicator arguments)");
                            m_checked[i + 1] = true;
                            T val = to<T>(m_argv[i + 1]);
                            Argument<T> ret(true);
                            ret.set(std::move(val));
                            return ret;
                        }
                    }
                }
                if constexpr (indicator) {
                    return Argument<T>(false);
                } else {
                    if (default_value) {
                        Argument<T> ret(true);
                        ret.set(*default_value);
                        return ret;
                    } else {
                        return Argument<T>(false);
                    }
                }
            }

        private:
            std::set<std::string> m_unique_keys;
            std::set<char> m_unique_flags;

            std::vector<bool> m_checked;
            int m_argc;
            char** m_argv;
            std::vector<Documentation> m_descriptions;
            bool m_help = false;
    };


}