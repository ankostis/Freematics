/**
 * Convert hex-string <--> >byte-array.
 *
 * From: https://stackoverflow.com/a/14051107/548792
 */
#include <iomanip>
#include <sstream>
#include <string>

void ncat_hex(std::stringstream &ss, const uint8_t *str, int len) {
  ss << std::hex;

  for (int i(0); i < len; ++i)
    ss << std::setw(2) << std::setfill('0') << (int)str[i];
}

void ncat_ascii(std::stringstream &ss, const char *str, int len) {
  int i = 0;
  char c;
  while ((c = *str++) && i++ < len) {
    ss << ((' ' <= c && c < 127) ? c : '.');
  }
}
