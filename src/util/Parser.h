//
// Created by jojo on 01.12.19.
//

#ifndef BAMAPPING_PARSER_H
#define BAMAPPING_PARSER_H

#include <fstream>
#include <sstream>
#include <map>

class Parser
{
public:
    Parser(std::string filename);
    void load(std::string filename);
    static std::string globalPoseGraphName();
    static std::string globalPoseGraphOptName();
    static std::string globalPoseGraphOptRefinedName();
    static std::string poseGraphName(size_t fragment_id);
    static std::string plyFileName(size_t fragment_id);
    static std::string FianalPlyName();
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


#endif //BAMAPPING_PARSER_H
