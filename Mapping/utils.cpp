#include "utils.hpp"


std::vector<std::string> split(const std::string& s, std::string regx) {
    
    std::vector<std::string> elems;
    std::regex re(regx); 
    std::sregex_token_iterator iter(s.begin(), s.end(), re, -1);
    std::sregex_token_iterator end;
    
    while (iter != end) {
        if (iter->length()) { elems.push_back(*iter); }
        ++iter;
    }

  return elems;
}