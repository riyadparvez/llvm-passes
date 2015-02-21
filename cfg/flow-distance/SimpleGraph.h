#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>

#include <map>
#include <vector>
#include <queue>
#include <set>

#include <stdint.h>

using namespace llvm;

struct Constraint {
  Constraint() : type(INVALID), variable(NULL), lastLoad(NULL), literal(0) { }
  bool operator==(const Constraint& other) const {
    return (type == other.type &&
      variable == other.variable &&
      lastLoad == other.lastLoad &&
      literal == other.literal);
  }
  enum Type { INVALID, SAT, UNSAT, EQ, NE };
  Type type;
  Value* variable;
  Instruction* lastLoad;
  int64_t literal;
  static Constraint getSat() {
    Constraint sat;
    sat.type = SAT;
    return sat;
  }
  static Constraint getUnsat() {
    Constraint unsat;
    unsat.type = UNSAT;
    return unsat;
  }
};

struct {
  std::string file_name;
  int start_line_no;
  int end_line_no;
  
  friend class boost::serialization::access;
  // When the class Archive corresponds to an output archive, the
  // & operator is defined similar to <<.  Likewise, when the class Archive
  // is a type of input archive the & operator is defined similar to >>.
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & file_name;
    ar & start_line_no;
    ar & end_line_no;
  }
};

std::vector<Constraint> getCommonConstraints(const std::vector<Constraint>& v1, const std::vector<Constraint>& v2) {
  std::vector<Constraint> result;
  for (std::vector<Constraint>::const_iterator i = v1.begin(), ie = v1.end(); i != ie; ++i) {
    for (std::vector<Constraint>::const_iterator i2 = v2.begin(), i2e = v2.end(); i2 != i2e; ++i2) {
      if (*i == *i2) {
        result.push_back(*i);
        break;
      }
    }
  }
  if (result.empty())
    result.push_back(Constraint::getSat());
  return result;
}


class Graph {
public:
struct Node {
std::vector<Constraint> vc;
};
struct Edge {
enum Type { Br, Call };
Type type;
BasicBlock* neighbor;
bool feasible;
Node* n;
Constraint c;
Instruction* call;
};
Graph() { }
~Graph() { }
void add(BasicBlock* src, BasicBlock* dst) {
add(src, dst, Graph::Edge::Br, NULL);
}
void add(BasicBlock* src, BasicBlock* dst,
Graph::Edge::Type type, Instruction* call) {
Node n;
if (nodes.find(src) == nodes.end())
nodes[src] = n;
if (nodes.find(dst) == nodes.end())
nodes[dst] = n;
std::vector<Edge>& edges = neighbors[src];
Edge e;
e.neighbor = dst;
e.n = &nodes[dst];
e.type = type;
e.call = call;
e.feasible = true;
e.c.type = Constraint::SAT;
edges.push_back(e);
}
void computeMinDist(BasicBlock* src, std::map<BasicBlock*, unsigned>& distances) {
  std::queue<BasicBlock*> q;
  std::set<BasicBlock*> visited;
  q.push(src);
  distances[src] = 0;
  visited.insert(src);
  while (!q.empty()) {
    BasicBlock* bb = q.front();
    q.pop();
    unsigned distance = distances[bb];
    std::vector<Edge> &nodes = neighbors[bb];
    for (std::vector<Edge>::iterator it = nodes.begin(), ite = nodes.end();
        it != ite; ++it) {
      if (visited.find(it->neighbor) == visited.end()) {
        visited.insert(it->neighbor);
        distances[it->neighbor] = distance + 1;
        q.push(it->neighbor);
      }
    }
  }
}
  std::vector<Edge>& getNeighbors(BasicBlock* b) { return neighbors[b]; }
  Node& getNode(BasicBlock* b) { return nodes[b]; }
private:
  std::map<BasicBlock*, std::vector<Edge> > neighbors;
  std::map<BasicBlock*, Node> nodes;
};
