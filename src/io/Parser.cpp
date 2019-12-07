//
// Created by jojo on 01.12.19.
//


#include "Parser.h"
using namespace BAMapping::io;

void Parser::load(std::string filename)
{
    std::ifstream file(filename);
    if(file.is_open())
    {
        while(!file.eof())
        {
            std::string line;
            if(std::getline(file,line))
            {
                std::istringstream line_str(line);
                std::string key;
                if(std::getline(line_str,key,':'))
                {
                    std::string value;
                    line_str>>std::ws;
                    if(std::getline(line_str,value))
                    {
                        mParamMap.emplace(key,value);
                    }
                }
            }
        }
    }
}
template<>
float Parser::getValue(std::string key)
{
    auto it = mParamMap.find(key);
    if(it != mParamMap.end())
    {
        return lexical_cast<float>(it->second);
    }
    else
    {
        return (float) 0;
    }
}

template<>
double Parser::getValue(std::string key)
{
    auto it = mParamMap.find(key);
    if(it != mParamMap.end())
    {
        return lexical_cast<double>(it->second);
    }
    else
    {
        return (double) 0;
    }
}

template<>
int Parser::getValue(std::string key)
{
    auto it = mParamMap.find(key);
    if(it != mParamMap.end())
    {
        return lexical_cast<int>(it->second);
    }
    else
    {
        return (int) 0;
    }
}

template<>
std::string Parser::getValue(std::string key)
{
    auto it = mParamMap.find(key);
    if(it != mParamMap.end())
    {
        return it->second;
    }
    else
    {
        return "";
    }
}
template<>
bool Parser::getValue(std::string key)
{
    auto it = mParamMap.find(key);
    if(it != mParamMap.end())
    {
        if(it->second == "true")
            return true;
        else if (it->second == "false")
            return false;
    }
    else
    {
        return false;
    }
}



