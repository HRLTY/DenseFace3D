#include <istream>
#include <string>
#include <vector>
#include <sstream>

std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str)
{
  std::vector<std::string> result;
  std::string line;
  std::getline(str, line);
  std::stringstream lineStream(line);
  std::string cell;

  while(std::getline(lineStream, cell, ',')){
    result.push_back(cell);
  }
  return result;
}
