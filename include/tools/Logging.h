#pragma once

#include <iostream>

#define AMP_LOG_COLOR
#define AMP_ASSERTS
#define AMP_DEBUG_TOOLS

#ifdef AMP_LOG_COLOR
    #define LOG(msg) std::cout << "\033[1;36m >[LOG]\033[0;37m " << msg << "\033[0m \n"
    #define PRINT(msg) std::cout << "\033[0;37m" << msg << "\033[0m \n"
    #define PRINT_VEC2(msg, vec2) std::cout << "\033[0;37m" << msg << " (" << vec2[0] << ", " << vec2[1] << ")\033[0m \n"
    #define PRINT_VEC3(msg, vec3) std::cout << "\033[0;37m" << msg << " (" << vec3[0] << ", " << vec3[1] << ", " << vec3[2] ")\033[0m \n"
    #define PRINT_NAMED(name, msg) std::cout << "\033[1;32m        "<< name << ": \033[0;37m" << msg << "\033[0m \n"
    #define DEBUG(msg) std::cout << "\033[1;36m >[DBG]("<< __func__ << "): \033[0;37m" << msg << "\033[0m \n"
    #define INFO(msg) std::cout << "\033[1;37m >[IFO] \033[0;37m" << msg << "\033[0m \n"
    #define ERROR(msg) std::cout << "\033[1;31m >[ERR] ERROR ("<< __func__ << "): \033[0;41m" << msg << "\033[0m \n"
    #define WARN(msg) std::cout << "\033[1;33m >[WRN] WARNING ("<< __func__ << "): \033[0;33m" << msg << "\033[0m \n"
    #define NEW_LINE std::cout<<"\n"
#elif AMP_LOG_NO_COLOR
    #define LOG(msg) std::cout << " >[LOG] " << msg << "\033[0m \n"
    #define PRINT(msg) std::cout <<  msg << "\n"
    #define PRINT_NAMED(name, msg) std::cout << "        "<< name << ": " << msg << "\n"
    #define DEBUG(msg) std::cout << " >[DBG] ("<< __func__ << "): " << msg << "\033[0m \n"
    #define ERROR(msg) std::cout << " >[ERR] ERROR ("<< __func__ << "): " << msg << "\n"
    #define WARN(msg) std::cout << " >[WRN] WARNING ("<< __func__ << "): " << msg << "\n"
    #define NEW_LINE std::cout<<"\n"
#else
    #define LOG(msg) 
    #define PRINT(msg) 
    #define PRINT_NAMED(name, msg) 
    #define DEBUG(msg)
    #define ERROR(msg)
    #define WARN(msg) 
    #define NEW_LINE 
#endif

#ifdef AMP_ASSERTS
    #define ASSERT(condition, msg) {if (!(condition)) {ERROR("[Assert fail] " << msg); exit(1);}}
#endif

#ifdef AMP_DEBUG_TOOLS
    #define PAUSE std::cout << "<paused>" << "\n"; \
            std::cin.get()
#endif

