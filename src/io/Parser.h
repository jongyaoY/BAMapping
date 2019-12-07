//
// Created by jojo on 01.12.19.
//

#ifndef BAMAPPING_PARSER_H
#define BAMAPPING_PARSER_H

#include <fstream>
#include <sstream>
#include <map>

namespace BAMapping
{
    namespace io
    {
        class Parser
        {
        public:
            void load(std::string filename);
            template <typename  T>
            T getValue(std::string key);
        private:
            std::map<std::string,std::string> mParamMap;

            template <typename T>
            T lexical_cast(const std::string& str)
            {
                T var;
                std::istringstream iss;
                iss.str(str);
                iss >> var;
                return var;
            }
        };
    }
}



#endif //BAMAPPING_PARSER_H
