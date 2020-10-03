/**
 * @file   ConfigParser_Lx16a.h
 * @author Michael Schmidpeter
 * @date   2019-10-25
 * @brief  todo
 * 
 * PROJECT: Franc0r
 * @see https://github.com/franc0r/Wiki/wiki
 */


#ifndef CONFIGPARSER_LX16A_H_
#define CONFIGPARSER_LX16A_H_

#include <string>
// #include <sstream>

#include "Servo_Lx16a.h"

#include "francor/XmlParser.h"


namespace francor::servo{

inline std::vector<Params_Lx16a> parseXML(const std::string& path)
{
  francor::XMLParser parser;
  parser.load(path);
  std::vector<Params_Lx16a> params;


  while(1)
  {
    try
    {
      parser.setFixChild("servo_config", "servo");
      Params_Lx16a p(
        /*p.id         :*/ std::stoi(parser.getChildText("id")),
        /*p.name       :*/ parser.getChildText("name"),
        /*p.base_frame :*/ parser.getChildText("base_frame"),
        /*p.max_v_in   :*/ std::stod(parser.getChildText("max_v_in")),
        /*p.min_v_in   :*/ std::stod(parser.getChildText("min_v_in")),
        /*p.max_temp   :*/ std::stoi(parser.getChildText("max_temp")),
        /*p.angle_max  :*/ std::stod(parser.getChildText("angle_max")),
        /*p.angle_min  :*/ std::stod(parser.getChildText("angle_min")),
        /*p.mode_servo :*/ (std::stoi(parser.getChildText("mode")) ? true : false)
      );

      //prove if id and name is unique
      bool id_name_unique = true;
      for(auto& e : params)
      {
        if(p.id == e.id || p.name == e.name)
        {
          id_name_unique = false;
          std::cout << "Warning given ID or Name not unique... ignore this servo" << std::endl;
          break;
        }
      }
      

      if(id_name_unique)
      {
        params.push_back(p);
      }

      parser.setFixChild("servo_config");
      parser.deleteChild("servo"); //get next servo
    }
    // catch(const std::invalid_argument& e)
    // {
    //   std::cerr << "Error at converting xml_file" <<e.what() << '\n';
    //   ::exit(EXIT_FAILURE);
    // }
    catch(const std::out_of_range& e)
    {
      break;
    }
  }

  std::cout << "Read config for " << params.size() << " servo(s) :" << std::endl;
  for(auto& e : params)
  {
    std::cout << e << std::endl;
  }

  return params;
}



} //namespace francor::servo


#endif  //CONFIGPARSER_LX16A_H_