#ifndef __SETTING_READER_HPP__
#define __SETTING_READER_HPP__

#include <string>
#include <map>
#include <set>


class SettingReader {
private:

    std::map<std::string,int> values;
    std::map<std::string,std::string> stringValues;

    // Used to bulk-parse
    template<int dim> void parseStringDoublePairsToDoubleArrays(std::array<double,dim> &destination,std::string varNameSuffix, std::array<std::string,dim> varNamePrefixes) {
        for (unsigned int i=0; i<dim; i++) {
            std::string currentKey = varNamePrefixes[i]+varNameSuffix;
            auto it = values.find(currentKey);
            if (it==values.end()) {
                std::ostringstream errorMsg;
                errorMsg << "Error in configuration file. Cannot find value for key '" << currentKey << "'";
                throw errorMsg.str();
            }
            destination[i] = it->second;
        }
    }
    template<int dim> void parseStringDoublePairsToIntArrays(std::array<int,dim> &destination,std::string varNameSuffix, std::array<std::string,dim> varNamePrefixes) {
        for (unsigned int i=0; i<dim; i++) {
            std::string currentKey = varNamePrefixes[i]+varNameSuffix;
            auto it = values.find(currentKey);
            if (it==values.end()) {
                std::ostringstream errorMsg;
                errorMsg << "Error in configuration file. Cannot find value for key '" << currentKey << "'";
                throw errorMsg.str();
            }
            destination[i] = it->second;
        }
    }

public:

    SettingReader(const char* filePath) {
        std::ifstream settingFile(filePath);
        if (settingFile.is_open()) {
            // Everything ok.
        }
        else {
            std::ostringstream os;
            os << "Error opening configuration file '" << filePath << "'. Please check that the file exists.";
            throw os.str();
        }

        std::string currentLine;
        while (std::getline(settingFile,currentLine)) {
            while (currentLine.substr(0,1)==" ") currentLine = currentLine.substr(1,std::string::npos);
            if (currentLine.size()==0) {
                // Empty line
            } else if (currentLine.substr(0,1)=="#") {
                // comment
            } else {
                size_t equalityPos=currentLine.find("=");
                if (equalityPos==std::string::npos) throw "Error in configuration file: Does not find the equality sign.";
                std::string key = currentLine.substr(0,equalityPos);
                std::string valueString = currentLine.substr(equalityPos+1,std::string::npos);

                std::istringstream valueReader(valueString);

                double value;
                valueReader >> value;
                if (valueReader.fail()) {
                    if (stringValues.count(key)>0) throw "Error: Value in configuration file multiply defined.";
                    stringValues[key] = currentLine.substr(equalityPos+1,std::string::npos);
                } else {
                    if (values.count(key)>0) throw "Error: Value in configuration file multiply defined.";
                    values[key] = value;
                    stringValues[key] = currentLine.substr(equalityPos+1,std::string::npos); // Because it may still be a string value
                }
            }
        }

        if (settingFile.bad()) throw "Error reading parameter file for the workspace properties.";
        settingFile.close();

    }

    int getValue(std::string valueName) {
        auto it = values.find(valueName);
        if (it==values.end()) {
            std::ostringstream errorMsg;
            errorMsg << "Error: Did not find integer value '" << valueName << "' in the settings file.";
            throw errorMsg.str();
        }
        return it->second;
    }

    std::string getStringValue(std::string valueName) {
        auto it = stringValues.find(valueName);
        if (it==stringValues.end()) {
            std::ostringstream errorMsg;
            errorMsg << "Error: Did not find string value '" << valueName << "' in the settings file.";
            throw errorMsg.str();
        }
        return it->second;
    }

};





#endif
