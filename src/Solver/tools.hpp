#ifndef __TOOLS_HPP__
#define __TOOLS_HPP__

#include <sstream>

inline std::vector<std::string> stringSplit(std::string baseString,char c) {
    std::vector<std::string> res;
    size_t lastBase = 0;
    for (size_t i=0; i<baseString.length(); i++) {
        if (baseString[i]==c) {
            res.push_back(baseString.substr(lastBase,i-lastBase));
            lastBase = i+1;
        }
    }
    res.push_back(baseString.substr(lastBase,baseString.length()));
    return res;
}

inline int toInt(std::string str) {
    std::istringstream is(str);
    int data;
    is >> data;
    if (is.bad()) {
        std::ostringstream os;
        os << "Cannot convert to int: " << str;
        throw os.str();
    } else {
        char c;
        is >> c;
        if (!is.eof()) {
            std::ostringstream os;
            os << "Cannot convert to int (stray characters): " << str;
            throw os.str();
        }
    }
    return data;
}

inline std::string toString(int a) {
    std::ostringstream os;
    os << a;
    return os.str();
}

int countAssignments(std::vector<BF> &ref, int refPos, BF data) {
    if (data.isFalse()) return 0;
    if (refPos==static_cast<int>(ref.size())) return 1;
    return countAssignments(ref,refPos+1,data & ref[refPos]) + countAssignments(ref,refPos+1,data & !ref[refPos]);
}

int safeDoubleToInt(double d) {
    int i = d;
    if (d!=i) throw "Error: Encountered a number that is not a safe integer value.";
    return i;
}











#endif
