/**
 * Convert hex string <--> >byte array.
 */
#include <sstream>
#include <iomanip>

void ncat_hex(std::stringstream &ss, const uint8_t *str, int len);
#define cat_hex(ss, str)  ncat_hex((ss), (str), sizeof(str))

void ncat_ascii(std::stringstream &ss, const char *str, int len);
#define cat_ascii(ss, str)  ncat_ascii((ss), (str), sizeof(str))