#include "readfile.h"

#include "Utils/logfile.h"
#include <iostream>
#include <fstream>

std::string readFile(std::string filePath) {
    std::string fileContent;
    std::ifstream fileStream(filePath.c_str(), std::ios::in);

    if(!fileStream.is_open()) {
		ERROR_LOG("Could not read file ");DEBUG_LOG(filePath);DEBUG_LOG(". File does not exist.\n");
        return "";
    }

    std::string line = "";
    while(!fileStream.eof()) {
        std::getline(fileStream, line);
        fileContent.append(line + "\n");
    }
    fileStream.close();
    
	return fileContent;
}