#ifndef NOP_H
#define NOP_H

#include <vector>
#include <set>
#include <map>
#include <string>

class TargetStatement
{
public:
  TargetStatement() : targetFilesCnt(0) { };
  ~TargetStatement() { };
  bool Load(const std::string& path);

  // Saves all alive lines covered 
  bool Save(const std::string& path);
  
  int inTarget(const std::string& file, int line) const;
  
  void alive(int locationID);
private:
  size_t targetFilesCnt;
  std::vector<std::string> targetFiles;

  // maps from actual target line to unique location ID
  std::vector<std::map<int, int> > targetLines;

  // Store the locations marked as executable code
  std::set<int> liveLocations;
};

#endif
