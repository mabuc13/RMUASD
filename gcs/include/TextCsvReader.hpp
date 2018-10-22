#ifndef TEXT_CSV_READER_H_
#define TEXT_CSV_READER_H_

#include <string>
#include <vector>
#include <map>
#include <locale>

using namespace std;

class CSVmsg{
public:
    CSVmsg(string text):CSVmsg(std::stringstream(text)){
    }
    CSVmsg(std::stringstream ss){
        text = ss.str();
        string comma;
        while(getline(ss,comma,',')){

            std::stringstream commaStream(comma);
            string name;
            getline(commaStream,name,'=');
            string value;
            getline(commaStream,value,'=');
            //cout << comma << " # " << name << " == " << value << endl;
            lowerCase(name);
            lowerCase(value);
            this->msg[name.c_str()] = value.c_str();
        }
    }
    string getText(void){
        return text;
    }
    bool hasValue(string name){
        try{
            lowerCase(name);
            this->msg.at(name);
            return true;
        }catch (const std::out_of_range& oor) {
            return false;
        }
    }
    string getValue(string name){
        try{
            lowerCase(name);
            return this->msg.at(name);
        }catch (const std::out_of_range& oor) {
            return "NULL";
        }
    }
    double getNumValue(string name){
        string value = getValue(name);
        //value.erase(remove_if(value.begin(), value.end(), isspace), value.end());
        try{
            //cout << "Double " << std::stod(value) << std::endl;
            return std::stod(value);
        }catch(std::invalid_argument e){

            cout << "Invalid argument: " << e.what() << endl;
            cout << "For value_" << value << "_"<<endl;
            return INFINITY;
        }


    }

private:
    void lowerCase(string & str){
        std::locale loc;
        for (std::string::size_type i=0; i<str.length(); ++i)
            str[i] = std::tolower(str[i],loc);
    }
    string text;
    std::map<string,string> msg;
};

#endif