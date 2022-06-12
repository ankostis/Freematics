#include "fsutil.h"

#include <FS.h>

#include <iomanip>
#include <sstream>

void listDir(File &root, std::stringstream &out, uint8_t levels) {
  if (!root) return;
  if (root.isDirectory()) {
    out << "DIR: " << root.path() << "\n";
    File file = root.openNextFile();
    while (file) {
      if (levels != 0) listDir(file, out, levels - 1);
      file = root.openNextFile();
    }
  } else {
    out << "+--> " << std::setw(11) << root.name() << " " << root.size()
        << "\n";
  }
}
