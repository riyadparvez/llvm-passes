/*
 * LLVM pass designed to filter out patch lines executed by the regression test suite
 * Only the remaining lines will be targeted by KATCH
 * By convention, a line is considered executed if at least one (low level) instruction
 * generated form it is executed (similar to gcov)
*/
#include <llvm/LLVMContext.h>
#include <llvm/Pass.h>
#include <llvm/Module.h>
#include <llvm/Type.h>
#include <llvm/GlobalVariable.h>
#include <llvm/Function.h>
#include <llvm/BasicBlock.h>
#include <llvm/DerivedTypes.h>
#include <llvm/Support/CallSite.h>
#include <llvm/IntrinsicInst.h>
#include <llvm/Analysis/DebugInfo.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>

#include <string>
#include <vector>
#include <map>
#include "TargetStatement.h"

using namespace llvm;


static cl::opt<std::string>
TargetFile("target-file",
           cl::init(""),
           cl::desc("Target filename"));

static cl::opt<std::string>
OutputFile("output-file",
           cl::init(""),
           cl::desc("Output filename"));

static cl::opt<bool>
StaticallyMinimize("statically-minimize",
                   cl::init(true),
                   cl::desc("Minimize the target by producing only one representative per basic block and removing non-executable lines"));

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
      if (ce->getOpcode()==Instruction::BitCast)
        if (Function *f = dyn_cast<Function>(ce->getOperand(0)))
          return f;
  
      // NOTE: This assert may fire, it isn't necessarily a problem and
      // can be disabled, I just wanted to know when and if it happened.
      assert(0 && "FIXME: Unresolved direct target for a constant expression.");
    }
  
    return 0;
  }

  Constant* stringToLLVM(Module& M, const std::string& str) {
    static std::map<std::string, Constant*> stringCache;
    if (stringCache.count(str))
      return stringCache[str];

    Constant* input = ConstantArray::get(M.getContext(), str, true);
    GlobalVariable* ginput = new GlobalVariable(/*Module=*/M,
      /*Type=*/ArrayType::get(IntegerType::get(M.getContext(), 8), str.size()+1),
      /*isConstant=*/true,
      /*Linkage=*/GlobalValue::PrivateLinkage,
      /*Initializer=*/0, // has initializer, specified below
      /*Name=*/(std::string)"." + str + ".mag1c");
    ginput->setAlignment(1);
    ginput->setInitializer(input);

    Constant* gepIndices[2];
    gepIndices[0] = ConstantInt::get(M.getContext(), APInt(64, StringRef("0"), 10));
    gepIndices[1] = ConstantInt::get(M.getContext(), APInt(64, StringRef("0"), 10));
    Constant* llvmStr = ConstantExpr::getGetElementPtr(ginput, gepIndices, 2);
    stringCache[str] = llvmStr;
    return llvmStr;
  }

  int inTarget(const TargetStatement& p, const Instruction *I) {
    std::string file;
    unsigned line;
    if (isa<IntrinsicInst>(I))
      return -1;
    if (getInstructionDebugInfo(I, file, line)) {
      //llvm::errs() << file << ":" << line << "\n";
      return p.inTarget(file, (int)line);
    } else {
      //llvm::errs() << "No debug information available\n";
    }
    return -1;
  }
 
  struct TargetFilter : public ModulePass {
  private:
    TargetStatement target;
    unsigned targetBBs, targetLines;

  Instruction* instrumentInit(Module& M) {
    Function* initFunction = M.getFunction("initFiltering");
    Function* userMain = M.getFunction("main");
    Instruction* firstInstruction = userMain->begin()->begin();

    Constant* inputFile = ConstantArray::get(M.getContext(), TargetFile, true);
    GlobalVariable* ginputFilename = new GlobalVariable(/*Module=*/M,
      /*Type=*/ArrayType::get(IntegerType::get(M.getContext(), 8), TargetFile.size()+1),
      /*isConstant=*/true,
      /*Linkage=*/GlobalValue::PrivateLinkage,
      /*Initializer=*/0, // has initializer, specified below
      /*Name=*/".strpfilterinfile");
    ginputFilename->setAlignment(1);
    ginputFilename->setInitializer(inputFile);

    Constant* gepIndices[2];
    gepIndices[0] = ConstantInt::get(M.getContext(), APInt(64, StringRef("0"), 10));
    gepIndices[1] = ConstantInt::get(M.getContext(), APInt(64, StringRef("0"), 10));
    Constant* argInFilename = ConstantExpr::getGetElementPtr(ginputFilename, gepIndices, 2);

    
    Constant* outputFile = ConstantArray::get(M.getContext(), OutputFile, true);
    GlobalVariable* goutputFilename = new GlobalVariable(/*Module=*/M,
      /*Type=*/ArrayType::get(IntegerType::get(M.getContext(), 8), OutputFile.size()+1),
      /*isConstant=*/true,
      /*Linkage=*/GlobalValue::PrivateLinkage,
      /*Initializer=*/0, // has initializer, specified below
      /*Name=*/".strpfilteroutfile");
    goutputFilename->setAlignment(1);
    goutputFilename->setInitializer(outputFile);

    Constant* argOutFilename = ConstantExpr::getGetElementPtr(goutputFilename, gepIndices, 2);

    std::vector<Value*> initArgs;
    initArgs.push_back(argInFilename);
    initArgs.push_back(argOutFilename);

    return CallInst::Create(initFunction, initArgs.begin(), initArgs.end(), "", firstInstruction);
  }

  void instrumentProgram(Module& M)
  {
    Function *initFunction = M.getFunction("initFiltering");
    
    if (!initFunction) {
      errs() << "Module does not contain exepected helper function. Ensure helper functions were linked against the module.\n";
      return;
    }
    if (!M.getFunction("main")) {
      errs() << "Module does not contain the main function. Unable to proceed\n";
      return;
    }
    
    Instruction* init = instrumentInit(M);

    std::map<BasicBlock*, int> BBtoId;
    std::set<std::pair<int, BasicBlock*> > locationsInstrumented;
    unsigned aliveInstructions = 0;

    for (Module::iterator f = M.begin(), fe = M.end(); f != fe; ++f) {
      for (Function::iterator b = f->begin(), be = f->end(); b != be; ++b) {
        bool BBInstrumented = false;
        for (BasicBlock::iterator i = b->begin(), ie = b->end(); i != ie; ++i) {
          int locationID = inTarget(target, &*i);
          if (locationID >= 0) {
            aliveInstructions++;
            target.alive(locationID);
            /* If we need to choose one instruction per basic block
             * check whether the current basic block has been covered
             * at a different location
             * */
            if (StaticallyMinimize &&
                BBtoId.count(&*b) &&
                BBtoId[&*b] != locationID) {
              // mark location as covered so it's not outputted
              // XXX this might be a problem if a location is not needed in the current BB
              // but is needed in the next one
              //target.covered(locationID);
            } else if (!locationsInstrumented.count(std::pair<int,BasicBlock*>(locationID, &*b))) {
              std::vector<Value*> args;
              std::string file;
              unsigned line = 0;
              getInstructionDebugInfo(i, file, line);
              Value* llvmLine = ConstantInt::get(Type::getInt32Ty(M.getContext()), line, false);
              args.push_back(stringToLLVM(M, file));
              args.push_back(llvmLine);
              BasicBlock::iterator ii, iie;
              for (ii = b->begin(), iie = b->end(); ii != iie; ++ii) {
                if (!isa<PHINode>(ii))
                  break;
              }

              BBtoId[&*b] = locationID;
              locationsInstrumented.insert(std::pair<int, BasicBlock*>(locationID, &*b));
              if (!BBInstrumented) {
                targetBBs++;
                BBInstrumented = true;
              }
            }
          }
        }
      }
    }
    errs() << "Found " << aliveInstructions << " live instructions\n";
  }
  public:
    static char ID;
    TargetFilter()
      : ModulePass(ID) 
      , targetBBs(0)
      , targetLines(0) {}

    virtual bool runOnModule(Module &M) {
      if (TargetFile.empty() || OutputFile.empty()) {
        errs() << "Both -target-file and -output-file must be specified\n";
        return false;
      }
      if (!target.Load(TargethFile)) {
        errs() << "Unable to load target file " << TargetFile << "\n";
        return false;
      }
      instrumentProgram(M);
      if (StaticallyMinimize)
        target.SaveUncoveredCode(OutputFile);
      errs() << "Target contains " << targetBBs << " basic blocks\n";
      return true;
    }
  };
}
  
char TargetFilter::ID = 0;
static RegisterPass<TargetFilter> X("targetfilter", "Target Filter", false, false);
