#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string>
#include <set>
#include <map>
#include <vector>
#include <fstream>
#include <queue>
#include <iostream>
#include "InstrumentationHelpers.h"

namespace {

  std::string _output;

  volatile uint64_t currentbb = 0;
  volatile bool indirect = false;
  std::set<uint64_t> lockedBB;

  struct BBInfo {
    BBInfo() : distance(-1), indegree(0) { }
    int distance;
    int indegree;
    std::set<uint64_t> neighbors;
  };
  std::map<uint64_t, BBInfo> graph;

  void addEdge(uint64_t src, uint64_t dst, int distancetodst) {
    BBInfo& gsrc = graph[src];
    if (gsrc.neighbors.find(dst) == gsrc.neighbors.end()) {
      gsrc.neighbors.insert(dst);
      BBInfo& gdst = graph[dst];
      gdst.indegree++;
      gdst.distance = distancetodst;
    }
  }

  int computeMinDistance()
  {
    std::map<uint64_t, BBInfo>::iterator n, ne;
    std::queue<BBInfo*> WorkList;
    for (n = graph.begin(), ne = graph.end(); n != ne; ++n)
      if (0 == n->second.indegree && lockedBB.find(n->first) == lockedBB.end())
        WorkList.push(&n->second);

    std::vector<BBInfo*>::iterator b, be;
    std::set<uint64_t>::iterator s, se;

    while (!WorkList.empty()) {
      BBInfo* bb = WorkList.front();
      WorkList.pop();
      for (s = bb->neighbors.begin(), se = bb->neighbors.end(); s != se; ++s) {
        if (lockedBB.find(*s) != lockedBB.end()) {
          std::cerr << "COVERAGE: WARNING: cycle involved locked basic block " << *s << "!\n";
          continue;
        }
        BBInfo& other = graph[*s];
        if (0 == --other.indegree)
          WorkList.push(&other);
      }
    }

    int minDist = -1;

    for (n = graph.begin(), ne = graph.end(); n != ne; ++n)
      if (0 != n->second.indegree && (minDist == -1 || n->second.distance < minDist))
        minDist = n->second.distance;
    return minDist;
  }

  static int savedMinDistance = 32000;
  void saveDistance()
  {
    int minDistance = computeMinDistance();
    if (-1 != minDistance && minDistance < savedMinDistance) {
      std::ofstream outf(_output.c_str());
      outf << minDistance;
      savedMinDistance = minDistance;
    }
  }
}

/*
 * Called by the instrumentation before executing main
 */
extern "C"
void nop_init(uint64_t mainbb)
{
  char wd[PATH_MAX];
  getcwd(wd, sizeof(wd));
  _output = wd;
  _output += "/dout";
  lockedBB.insert(mainbb);
  atexit(saveDistance);

  currentbb = 0;
}

std::set<std::pair<uint64_t, uint64_t> > breakEdges;

/*
 * Called by the instrumentation whenever execution reaches a new
 * basic block
 */
extern "C"
void nop_transition(uint64_t bb, int distance)
{
  // save initial value. globals might be modified by calls
  bool my_indirect = indirect;
  uint64_t my_currentbb = currentbb;

  if (my_indirect) {
    lockedBB.insert(my_currentbb);
  }
  if (my_currentbb &&
      !breakEdges.count(std::pair<uint64_t, uint64_t>(my_currentbb, bb))) {
    addEdge(my_currentbb, bb, distance);
    // need to explicitly call this in case of execxx or _exit
    if (distance < savedMinDistance)
      saveDistance();
  }
  indirect = false;
  currentbb = bb;
}

/*
 * Called by the instrumentation  after each function to reset the current
 * basic block
 */
extern "C"
void nop_bb(uint64_t bb)
{
  currentbb = bb;
  indirect = false;
}

/*
 * Called by the instrumentation before all indirect function calls
 */
extern "C"
void nop_indirect_call()
{
  indirect = true;
}

/*
 * Called by the instrumentation before executing main for each
 * edge which was determined to be infeasible
 */
extern "C"
void nop_break_edge(uint64_t src, uint64_t dst) {
  breakEdges.insert(std::pair<uint64_t, uint64_t>(src, dst));
}

