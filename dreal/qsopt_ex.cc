/// @file qsopt_ex.cc
///

#include "dreal/qsopt_ex.h"

using std::string;

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

}  // namespace qsopt_ex
}  // namespace dreal
