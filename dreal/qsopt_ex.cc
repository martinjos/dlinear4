/// @file qsopt_ex.cc
///

#include "dreal/qsopt_ex.h"

using std::string;
using std::fprintf;
using std::exit;

// Needed for mpq_EGlpNumAllocArray(), etc.
#define EXIT(__A, ...)                             \
    do {                                           \
        if (__A) {                                 \
            fprintf(stderr, "EXIT: " __VA_ARGS__); \
            exit(EXIT_FAILURE);                    \
        }                                          \
    } while (0)

namespace dreal {
namespace qsopt_ex {

mpq_class* StringToMpqPtr(const string& str) {
    mpq_t val;
    mpq_init(val);
    mpq_EGlpNumReadStr(val, str.c_str());
    mpq_class* result = new mpq_class(val);
    mpq_clear(val);
    return result;
}

mpq_class StringToMpq(const string& str) {
    mpq_t val;
    mpq_init(val);
    mpq_EGlpNumReadStr(val, str.c_str());
    mpq_class result(val);
    mpq_clear(val);
    return result;
}

MpqArray::MpqArray(int size) {
    array = mpq_EGlpNumAllocArray(size);
}

MpqArray::~MpqArray() {
    mpq_EGlpNumFreeArray(array);
}

void QSXStart() {
  QSexactStart();
}

void QSXFinish() {
  QSexactClear();
}

}  // namespace qsopt_ex
}  // namespace dreal
