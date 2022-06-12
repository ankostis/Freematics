#pragma once

#include <FS.h>

#include <iomanip>
#include <sstream>

void listDir(File &root, std::stringstream &out, uint8_t levels = -1);
