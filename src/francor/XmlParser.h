#ifndef XMLPARSER_H_
#define XMLPARSER_H_




#include <string>
#include <stdexcept>
#include <tinyxml2.h>

namespace francor {

class XMLParser {
public:
  void load(const std::string& file) //throw (std::out_of_range)
  {
    _doc.LoadFile(file.c_str());
    if(_doc.ErrorID() != 0)
    {
      throw std::out_of_range("Error at reading given XML-File: " + file + ", error-message: " + _doc.ErrorName());
    }
  }

  template<typename ...Args>
  std::string getChildText(Args&... args)  //throw (std::out_of_range)
  {
    std::initializer_list<std::string> childnodes{ args... };
    tinyxml2::XMLElement* element = nullptr;

    for(auto& e : childnodes)
    {
      if(element)
        element = element->FirstChildElement(e.c_str());
      else if(_fixelement)
        element = _fixelement->FirstChildElement(e.c_str());
      else
        element = _doc.FirstChildElement(e.c_str());


      if(element == nullptr)
      {
        throw std::out_of_range(std::string("given child does not exist ->") + e);
      }
    }

    return std::string(element->GetText());
  }

  template<typename ...Args>
  void setFixChild(Args&... args)  //throw (std::out_of_range)
  {
    std::initializer_list<std::string> childnodes{ args... };
    _fixelement = nullptr;

    for(auto& e : childnodes)
    {
      if(_fixelement)
        _fixelement = _fixelement->FirstChildElement(e.c_str());
      else
        _fixelement = _doc.FirstChildElement(e.c_str());
      if(_fixelement == nullptr)
      {
        throw std::out_of_range(std::string("given fix-Child does not exist ->") + e);
      }
    }
  }

  template<typename ...Args>
  void deleteChild(Args&... args) //throw (std::out_of_range)
  {
    std::initializer_list<std::string> childnodes{ args... };
    tinyxml2::XMLElement* element = nullptr;

    for(auto& e : childnodes)
    {
      if(element)
        element = element->FirstChildElement(e.c_str());
      else if(_fixelement)
        element = _fixelement->FirstChildElement(e.c_str());
      else
        element = _doc.FirstChildElement(e.c_str());

      if(element == nullptr)
      {
        throw std::out_of_range(std::string("given child does not exist ->") + e);
      }
    }

    //delete
    auto parent = element->Parent();
    parent->DeleteChild(element);
  }

private:
  tinyxml2::XMLDocument _doc;
  tinyxml2::XMLElement* _fixelement = nullptr;
};

}  // namespace francor

#endif  //XMLPARSER_H_

