#include "TargetStatement.h"
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdlib.h>

using namespace std;

bool TargetStatement::Load(const string& path)
{
  ifstream inf(path.c_str());
  if (!inf.is_open())
    return false;

  string file, lines;
  map<int, int> lineset;
  int locationID = 0;

  while (!inf.eof()) {
    getline(inf, file);
    getline(inf, lines);
    if (file.empty())
      continue;

    //cerr << "file: " << file << endl;
    stringstream strstr(lines);
    istream_iterator<std::string> it(strstr);
    istream_iterator<std::string> end;
    lineset.clear();
    for (; it != end; ++it) {
      lineset[atoi(it->c_str())] = locationID++;
      //cerr << atoi(it->c_str()) << " ";
    }
    //cerr << endl;

    targetFiles.push_back(file);
    targetLines.push_back(lineset);
  }
  targetFilesCnt = targetFiles.size();
  return true;
}

// Find a suitable name
bool TargetStatement::Save(const string& target)
{
  ofstream outf(target.c_str());
  if (!outf.is_open())
    return false;

  for (size_t i = 0; i < pathFilesCnt; ++i) {
    if (coveredCPPatchLines[i].size()) {
      outfcp << patchFiles[i] << endl;
      for (set<int>::const_iterator it = coveredCPPatchLines[i].begin(), ite = coveredCPPatchLines[i].end();
          it != ite; ++it)
        outfcp << *it << " ";
      outfcp << endl;
    }
  }
  
  return true;
}

int TargetStatement::inTarget(const string& file, int line) const
{
  if (!file.size() || !line)
    return -1;
  size_t i;
  for (i = 0; i < targetFilesCnt; ++i) {
    if (file.find(targetFiles[i]) != string::npos)
      return ((targetLines[i].find(line) != targetLines[i].end()) ?
              targetLines[i].find(line)->second :
              -1);
  }
  return -1;
}

void TargetStatement::alive(int locationID)
{
  if (locationID < 0)
    return;
  liveLocations.insert(locationID);
}
