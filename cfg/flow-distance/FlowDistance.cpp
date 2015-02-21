#include <llvm/IR/LLVMContext.h>
#include <llvm/Pass.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Constants.h>
#include <llvm/Support/CallSite.h>
#include <llvm/DebugInfo.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Analysis/ConstantFolding.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <string>
#include <vector>
#include "SimpleGraph.h"

using namespace llvm;


static cl::opt<std::string>
TargetFile("target-file",
           cl::init(""),
           cl::desc("Target filename"));

static cl::opt<unsigned>
TargetLine("target-line",
           cl::init(0),
           cl::desc("Target line number"));

namespace {
  std::string getDSPIPath(DILocation Loc) {
    std::string dir = Loc.getDirectory();
    std::string file = Loc.getFilename();
  
    if (dir.empty() || file[0] == '/') {
      return file;
    } else if (*dir.rbegin() == '/') {
      return dir + file;
    } else {
      return dir + "/" + file;
    }
  }

  bool getInstructionDebugInfo(const llvm::Instruction *I,
                                                     std::string &File,
                                                     unsigned &Line) {
    if (MDNode *N = I->getMetadata("dbg")) {
      DILocation Loc(N);
      File = getDSPIPath(Loc);
      Line = Loc.getLineNumber();
      return true;
    }
    return false;
  }

  Function *getDirectCallTarget(CallSite cs) {
    Value *v = cs.getCalledValue();
    if (Function *f = dyn_cast<Function>(v)) {
      return f;
    } else if (llvm::ConstantExpr *ce = dyn_cast<llvm::ConstantExpr>(v)) {
      if (ce->getOpcode()==Instruction::BitCast) {
        if (Function *f = dyn_cast<Function>(ce->getOperand(0))) {
          return f; 
        }
      }
      // NOTE: This assert may fire, it isn't necessarily a problem and
      // can be disabled, I just wanted to know when and if it happened.
      assert(0 && "FIXME: Unresolved direct target for a constant expression.");
    }
  
    return 0;
  }

  bool isIntrinsicCall(Instruction* i) {
    Function *f = getDirectCallTarget(i);
    return (f ? f->isIntrinsic() : false);
  }
 
  struct Coverage : public ModulePass {
  private:
    std::string targetFile;
    unsigned targetLine;
    Graph g;
    std::map<BasicBlock*, unsigned> distances;
    std::vector<std::pair<BasicBlock*, BasicBlock*> > toErase;

  bool getTarget() {
    if (TargetFile.empty() || !TargetLine) {
      errs() << "Please specify --target-file and --target-line\n";
      return false;
    }
    targetFile = TargetFile;
    targetLine = TargetLine;
    return true;
  }

  Constraint translateConstraint(const Constraint& c, BasicBlock* bb, Instruction* call, Function* targetFn) {
    // c.variable is the only thing which may change
    //errs() << "translateConstraints() for basic block" << *bb << "\n";
    if (c.type == Constraint::SAT || c.type == Constraint::UNSAT)
      return c;
    Constraint nc = c;
    assert(nc.type != Constraint::INVALID && "invalid constraint");
    assert(nc.variable && "constraint with NULL variable");

    //errs() << "Translating variable " << *nc.variable << "\n";
    //errs() << (call ? "Function call" : "NO function call") << "\n";

    Value* v = nc.variable, *prevv = v;

    if (PHINode* phi = dyn_cast<PHINode>(v)) {
      v = phi->getIncomingValueForBlock(bb);
    }
    if (call) {
      assert((isa<CallInst>(call) || isa<InvokeInst>(call)) && "unrecognized call instruction");
      assert(targetFn && "no target for function call given");
      // transform v into the corresponding argument
      int argNo = -1;
      for (Function::arg_iterator a = targetFn->arg_begin(), ae = targetFn->arg_end(); a != ae; ++a) {
        if (a == v) {
          argNo = a->getArgNo();
          break;
        }
      }
      if (argNo != -1) {
        CallSite cs(call);
        v = cs.getArgument(argNo);
      }
    }
    while (v) {
      Instruction* i;
      LoadInst*    li;
      StoreInst*   si;
      Constant*    ci;
      prevv = v;
      if ((i = dyn_cast<Instruction>(v))) {
        if (isa<PHINode>(v)) {
          // will be resolved when processing parent bb
          break;
        } else if ((ci = ConstantFoldInstruction(i))) {
          v = ci;
          break;
        } else if ((li = dyn_cast<LoadInst>(i))) {
          nc.lastLoad = i;
          v = li->getPointerOperand();
        } else if (isa<AllocaInst>(v)) {
          if (nc.lastLoad) {
            // search for a store
            BasicBlock::InstListType::reverse_iterator ii, iie;
            ii = bb->getInstList().rbegin(), iie = bb->getInstList().rend();
            if (nc.lastLoad->getParent() == bb) {
              for (; ii != iie && &*ii != nc.lastLoad; ++ii) { /* empty */ }
              if (ii != iie) ++ii;
            }
            for (; ii != iie; ++ii) {
              if ((si = dyn_cast<StoreInst>(&*ii)) && si->getPointerOperand() == v) {
                v = si->getValueOperand();
                nc.lastLoad = NULL;
                break;
              } else if (isa<LoadInst>(&*ii)) {
                // empty
              } else {
                // all other operations on v are unsupported
                for (unsigned op = 0, cop = ii->getNumOperands(); op < cop; ++op) {
                  if (ii->getOperand(op) == v) {
                    //errs() << "Unsupported operation " << *ii << "\n";
                    v = NULL;
                    break;
                  }
                }
              }
            }
            if (ii == iie) {
              break;
            }
          } else {
            //XXX: I'm lost
            v = NULL;
            break;
          }
        } else {
          //errs() << "Don't know how to process instruction " << *i << "\n";
          v = NULL;
        }
        // make sure we don't go out of the current bb
        if (v && (i = dyn_cast<Instruction>(v)) && i->getParent() != bb) {
          v = prevv;
          break;
        }
      } else if (isa<Constant>(v)) {
        break;
      } else {
        // might be a function argument
        break;
      }
    };

    if (v) {
      nc.variable = v;
    } else {
      nc.type = Constraint::SAT;
    }
    /*
    if (v)
      errs() << "translateConstraints() ends; variable is now " << *v
             << (nc.type == Constraint::EQ ? " EQ " : (nc.type == Constraint::NE ? " NE " : " UNKOP ")) << nc.literal << "\n";
    else
      errs() << "translateConstraints() ends; variable is now SAT\n";
    */
    return nc;
  }

  // Same constraints
  int compareConstraints(const std::vector<Constraint>& lhs, const std::vector<Constraint>& rhs) {
    if (lhs.size() != rhs.size())
      return 1;
    std::vector<Constraint>::const_iterator i1, i1e, i2, i2e;
    for (i1 = lhs.begin(), i1e = lhs.end(),
         i2 = rhs.begin(), i2e = rhs.end();
         i1 != i1e; ++i1, ++i2) {
      if (i1->type != i2->type ||
          i1->variable != i2->variable ||
          i1->type != i2->type)
        return 1;
    }
    return 0;
  }

  // Add edges between BBs to CFG
  void addEdges(Module& M, Graph& g) {
    for (Module::iterator f = M.begin(), fe = M.end(); f != fe; ++f) {
      for (Function::iterator b = f->begin(), be = f->end(); b != be; ++b) {
        BasicBlock* bb = b;
        TerminatorInst* ti = bb->getTerminator();
        
        if (BranchInst *branch = dyn_cast<BranchInst>(ti)) {
          if (branch->isUnconditional()) {
            continue;
          }
          Value* v = branch->getCondition();
          ICmpInst* icmp = dyn_cast<ICmpInst>(v);
          if (icmp && icmp->isEquality()) { // EQ or NE
            ConstantInt* literal = dyn_cast<ConstantInt>(icmp->getOperand(0));
            LoadInst* variable = dyn_cast<LoadInst>(icmp->getOperand(1));
            if (!literal || !variable) {
              literal = dyn_cast<ConstantInt>(icmp->getOperand(1));
              variable = dyn_cast<LoadInst>(icmp->getOperand(0));
              if (!literal || !variable)
                continue;
            }

            Constraint cx[2];
            cx[0].variable = cx[1].variable = variable;
            cx[0].literal = cx[1].literal = literal->getSExtValue();
            if (icmp->getPredicate() == CmpInst::ICMP_EQ) {
              cx[0].type = Constraint::EQ;
              cx[1].type = Constraint::NE;
            } else {
              cx[0].type = Constraint::NE;
              cx[1].type = Constraint::EQ;
            }

            assert(branch->getNumSuccessors() == 2 && "unexpected number of successors");
            for (unsigned cnt = branch->getNumSuccessors(); cnt; --cnt) {
              BasicBlock* target = branch->getSuccessor(cnt-1);
              std::vector<Graph::Edge>& edges = g.getNeighbors(target);
              std::vector<Graph::Edge>::iterator e, ei;
              for (e = edges.begin(), ei = edges.end(); ei != e; ++e) {
                if (e->neighbor == bb)
                  break;
              }
              assert (e != ei && "corresponding edge not found in graph");
              e->c = cx[cnt-1];
            }
          }
        } else if (SwitchInst *si = dyn_cast<SwitchInst>(ti)) {
          Constraint c;
          c.variable = si->getCondition();
          c.type = Constraint::EQ;
          for (unsigned ci = 1, cie = si->getNumCases(); ci < cie; ++ci) {
            c.literal = si->getCaseValue(ci)->getSExtValue();
            BasicBlock* target = si->getSuccessor(ci);
            std::vector<Graph::Edge>& edges = g.getNeighbors(target);
            std::vector<Graph::Edge>::iterator e, ei;
            for (e = edges.begin(), ei = edges.end(); ei != e; ++e) {
              if (e->neighbor == bb)
                break;
            }
            assert (e != ei && "corresponding edge not found in graph");
            e->c = c;
          }
        }
      }
    }
  
  }
  
  bool isUnsat(const std::vector<Constraint>& vc) {
    std::vector<Constraint>::const_iterator i, ie;
    for (i = vc.begin(), ie = vc.end(); i != ie; ++i) {
      if (i->type == Constraint::UNSAT)
        return true;
    }
    return false;
  }

  bool isSat(const std::vector<Constraint>& vc) {
    return (1 == vc.size() && vc[0].type == Constraint::SAT);
  }

  void pruneCFG(Module& M, Graph& g, BasicBlock* target) {
    addEdges(M, g);

    Constraint c;
    c.type = Constraint::SAT;
    Graph::Node& tn = g.getNode(target);
    tn.vc.push_back(c);

    //errs() << "starting from bb " << *target << "\n";
    std::set<BasicBlock*> WorkList;
    std::set<BasicBlock*>::iterator wli;
    WorkList.insert(target);

    do {
      wli = WorkList.begin();
      BasicBlock* bb = *wli;
      WorkList.erase(wli);

      Graph::Node& n = g.getNode(bb);
      if (n.vc.size()) {
        std::vector<Graph::Edge>& edges = g.getNeighbors(bb);
        for (std::vector<Graph::Edge>::iterator e = edges.begin(),
             ee = edges.end(); e != ee; ++e) {
          std::vector<Constraint> vc;
          for (std::vector<Constraint>::iterator v = n.vc.begin(), ve = n.vc.end();
               v != ve; ++v) {
            if (v->type != Constraint::SAT)
              vc.push_back(translateConstraint(*v, e->neighbor, (e->type == Graph::Edge::Call ? e->call : NULL), bb->getParent()));
          }
          if (e->c.type != Constraint::SAT) // the branch condition may be simplified or could be a call
            vc.push_back(translateConstraint(e->c, e->neighbor, (e->type == Graph::Edge::Call ? e->call : NULL), bb->getParent()));
          else
            vc.push_back(e->c);
          for (std::vector<Constraint>::iterator v = vc.begin(), ve = vc.end();
               v != ve; ++v) {
            if ((v->type == Constraint::EQ || v->type == Constraint::NE) &&
                isa<Constant>(v->variable)) {
              ConstantInt* ci;
              if ((ci = dyn_cast<ConstantInt>(v->variable))) {
                int64_t value = ci->getSExtValue();
                if (v->type == Constraint::EQ)
                  v->type = (value == v->literal ? Constraint::SAT : Constraint::UNSAT);
                else
                  v->type = (value != v->literal ? Constraint::SAT : Constraint::UNSAT);
              } else {
                //errs() << "Don't know how to process constant: " << *v->variable << "\n";
              }
            }
          }
          if (isUnsat(vc)) {
            // simplify it
            //errs() << "Found unsat edge with " << vc.size() << " conjuncts\n";
            vc.clear();
            vc.push_back(Constraint::getUnsat());
            e->feasible = false;
          } else {
            e->feasible = true;
          }

          if (e->n->vc.empty()) { // not visited
            e->n->vc = vc;
            WorkList.insert(e->neighbor);
          } else { // already visited
            if (isUnsat(vc) || isSat(e->n->vc)) {
                // nothing
            } else if (isUnsat(e->n->vc)) {
              e->n->vc = vc;
              WorkList.insert(e->neighbor);
            } else { // different
              size_t csize = e->n->vc.size();
              e->n->vc = getCommonConstraints(vc, e->n->vc);
              if (csize != e->n->vc.size() || isSat(e->n->vc)) {
                WorkList.insert(e->neighbor);
              }
            }
          }
        }
      }
    } while (WorkList.size());

    unsigned deadbb = 0, deade = 0;
    for (Module::iterator f = M.begin(), fe = M.end(); f != fe; ++f) {
      for (Function::iterator b = f->begin(), be = f->end(); b != be; ++b) {
        BasicBlock* bb = &*b;
        std::vector<Graph::Edge>& edges = g.getNeighbors(bb);
        std::set<BasicBlock*> infeasibleNeighbors;
        // it is a multigraph
        for (std::vector<Graph::Edge>::iterator e = edges.begin(),
                ee = edges.end(); e != ee; ++e) {
          if (!e->feasible) {
            infeasibleNeighbors.insert(e->neighbor);
          }
        }
        for (std::vector<Graph::Edge>::iterator e = edges.begin(),
                ee = edges.end(); e != ee; ++e) {
          if (e->feasible)
            infeasibleNeighbors.erase(e->neighbor);
        }
        for (std::set<BasicBlock*>::const_iterator f = infeasibleNeighbors.begin(), fe = infeasibleNeighbors.end(); f != fe; ++f) {
          toErase.push_back(std::pair<BasicBlock*, BasicBlock*>(*f, bb));
          errs() << "Edge removed: " << **f << " to " << *bb << "\n";
          ++deade;
        }
        Graph::Node& n = g.getNode(bb);
        if (isUnsat(n.vc)) {
          errs() << "Basic block unreach" << *bb << "\n";
          ++deadbb;
        }
      }
    }
    errs() << "Removed " << deadbb << " basic blocks and " << deade << " edges\n";
  }

  BasicBlock* computeDistances(Module& M) {
    BasicBlock* src = NULL;
    for (Module::iterator f = M.begin(), fe = M.end(); f != fe; ++f) {
      for (Function::iterator b = f->begin(), be = f->end(); b != be; ++b) {
        for (BasicBlock::iterator i = b->begin(), ie = b->end(); i != ie; ++i) {
          // is this the target?
          std::string file;
          unsigned line;
          if (getInstructionDebugInfo(i, file, line)) {
            if (line == targetLine && file.find(targetFile) != std::string::npos) {
              // some lines may spawn multiple basic blocks. don't choose a trivial one
              if (b->size() > 1)
                src = &*b;
            }
          }
          // construct the reverse CFG
          // XXX: reimplement via common TerminatorInst interface
          Instruction *I = &*i;
          if (BranchInst *branch = dyn_cast<BranchInst>(I)) {
            for (unsigned cnt = branch->getNumSuccessors(); cnt; --cnt) {
              g.add(branch->getSuccessor(cnt-1), &*b);
            }
          } else if (SwitchInst *si = dyn_cast<SwitchInst>(I)) {
            for (unsigned cnt = si->getNumCases(); cnt; --cnt) {
              g.add(si->getSuccessor(cnt-1), &*b);
            }
          } else if (isa<CallInst>(I) || isa<InvokeInst>(I)) {
            Function *target = getDirectCallTarget(I);
            if (target) {
              if (target->isIntrinsic())
                continue;
              g.add(&*target->begin(), &*b, Graph::Edge::Call, I);
            } else {
              //TODO decide how to handle indirect calls
            }
            InvokeInst* inv = dyn_cast<InvokeInst>(I);
            if (inv) {
              g.add(inv->getNormalDest(), &*b);
              g.add(inv->getUnwindDest(), &*b);
            }
          }
        }
      }
    }
    if (!src) {
      errs() << "Could not find target\n";
    } else {
      pruneCFG(M, g, src);
      g.computeMinDist(src, distances);
    }
    return src;
  }

  public:

    static char ID;
    Coverage() : ModulePass(ID) {}

    virtual bool runOnModule(Module &M) {
      if (!getTarget())
        return false;
      BasicBlock* target = computeDistances(M);
      return true;
    }

  };
}
  
char Coverage::ID = 0;
// Register pass for llvm
static RegisterPass<Coverage> X("coverage", "Coverage Instrumentation", false, false);
