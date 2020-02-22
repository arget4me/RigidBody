#ifndef LOG_FILE_HEADER
#define LOG_FILE_HEADER

#if 1

#include <iostream>
#define DEBUG_LOG(x) { \
	std::cout << x; \
}

#define ERROR_LOG(x) { \
	std::cout << "ERROR: " << x; \
}

#define DEBUG_CALL(x) x

#else

#define DEBUG_LOG(x) 

#define DEBUG_CALL(x) 

#endif



#endif