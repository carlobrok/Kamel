#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H

#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <opencv2/opencv.hpp>


namespace configuration
{

  //---------------------------------------------------------------------------
  // The configuration::data is a simple map string (key, value) pairs.
  // The file is stored as a simple listing of those pairs, one per line.
  // The key is separated from the value by an equal sign '='.
  // Commentary begins with the first non-space character on the line a hash or
  // semi-colon ('#' or ';').
  //
  // Example:
  //   # This is an example
  //   source.directory = C:\Documents and Settings\Jennifer\My Documents\
  //   file.types = *.jpg;*.gif;*.png;*.pix;*.tif;*.bmp
  //
  // Notice that the configuration file format does not permit values to span
  // more than one line, commentary at the end of a line, or [section]s.
  //

  struct data: std::map <std::string, std::pair<std::string, unsigned int>> {

    std::vector<std::string> line_strings;

    // Here is a little convenience method...
    bool iskey( const std::string& s ) const {
      return count( s ) != 0;
    }

    std::string getvalue( const std::string& key ) {
      return (this->operator [] ( key )).first;
    }

    // Gets an integer value from a key. If the key does not exist, or if the value
     // is not an integer, throws an int exception.
     //
    int getintvalue( const std::string& key ) {
       if (!iskey( key )) throw 0;
       std::istringstream ss( getvalue(key) );
       int result;
       ss >> result;
       if (!ss.eof()) throw 1;
       return result;
    }

    bool getboolvalue( const std::string& key ) {
      if(!iskey( key )) throw 0;
      std::string value( getvalue(key) );
      return (value == "true") || (value == "1");
    }

    cv::Scalar getscalarvalue( const std::string& key ) {
      if(!iskey( key )) throw 0;

      std::vector<int> vect;
      std::stringstream ss( getvalue(key) );

      for (int i; ss >> i;) {
          vect.push_back(i);
          if (ss.peek() == ',')
              ss.ignore();
      }

      if (vect.size() != 3) throw 1;

      return cv::Scalar(vect[0], vect[1], vect[2]);
    }

    // Template für alle Variablen, die Ganzzahlen, oder Gleitkommazahlen sind.
    // https://stackoverflow.com/questions/44848011/c-limit-template-type-to-numbers
    template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* = nullptr>
    void setvalue( const std::string& key, T value ) {
      if (!iskey( key )) throw 0;
      iterator it = find(key);
      if(it != end())
        it->second.first = std::to_string(value);
      else
        throw 1;
    }

    //
    void setvalue( const std::string& key, bool value ) {
      if (!iskey( key )) throw 0;
      iterator it = find(key);
      if(it != end())
        it->second.first = (value ? "true" : "false");
      else
        throw 1;
    }

    void setvalue( const std::string& key, cv::Scalar value ) {
      if (!iskey( key )) throw 0;
      iterator it = find(key);
      if(it != end()) {
        std::string value_string = "";
        value_string += std::to_string((int)value[0]) + ",";
        value_string += std::to_string((int)value[1]) + ",";
        value_string += std::to_string((int)value[2]);
        it->second.first = value_string;
      }
      else
        throw 1;
    }

  };

  //---------------------------------------------------------------------------
  // The extraction operator reads configuration::data until EOF.
  // Invalid data is ignored.
  //
  std::istream& operator >> ( std::istream& ins, data& d )
  {
    std::string s, key, value;

    // For each (key, value) pair in the file

    for (unsigned int line = 0; std::getline( ins, s ); line++)
    {
      d.line_strings.push_back(s);
      std::string::size_type begin = s.find_first_not_of( " \f\t\v" );

      // Skip blank lines
      if (begin == std::string::npos) continue;

      // Skip commentary
      if (std::string( "#;" ).find( s[ begin ] ) != std::string::npos) continue;

      // Extract the key value
      std::string::size_type end = s.find( '=', begin );
      key = s.substr( begin, end - begin );

      // (No leading or trailing whitespace allowed)
      key.erase( key.find_last_not_of( " \f\t\v" ) + 1 );

      // No blank keys allowed
      if (key.empty()) continue;

      // Extract the value (no leading or trailing whitespace allowed)
      begin = s.find_first_not_of( " \f\n\r\t\v", end + 1 );
      end   = s.find_last_not_of(  " \f\n\r\t\v" ) + 1;

      value = s.substr( begin, end - begin );

      // Insert the properly extracted (key, value) pair into the map
      d[ key ] = std::make_pair(value, line);
    }

    std::cout << "in line_strings after reading file" << std::endl;
    //for( const auto & line: d.line_strings) {
    //  std::cout << "line:" << line << std::endl;
    //}
    std::cout << std::endl;


    return ins;
  }

  //---------------------------------------------------------------------------
  // The insertion operator writes all configuration::data to stream.
  //
  std::ostream& operator << ( std::ostream& outs, const data& d )  {
    std::vector<std::string> m_line_strings = d.line_strings;

    std::cout << "in m_line_strings:" << std::endl;
    for( auto & line: m_line_strings) {
      std::cout << line << std::endl;
    }
    std::cout << std::endl << "Updating lines" << std::endl;

    for (auto const& x : d) {
      std::string new_line = (x.first + " = " + x.second.first);
      std::cout << "Line " << x.second.second << " = '" << new_line << "'" << std::endl;
      m_line_strings.at(x.second.second) = new_line;
    }

    std::cout << "Writing to file" << std::endl;

    for( auto & line: m_line_strings) {
      outs << line << std::endl;
    }
    return outs;
  }

} // namespace configuration

#endif
