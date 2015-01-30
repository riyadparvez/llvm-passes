#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include "InstrumentationHelpers.h"
#include "../Patch.h"

/*
 * Called by the instrumentation before executing main
 */
extern "C"
void initFiltering(const char* input, const char* output)
{
  if (output[0] != '/') {
    getcwd(wd, sizeof(wd));
    _output = wd;
    _output += "/";
  }
  _output += output;
  target.Load(input);
  //atexit(finalizeFiltering);
}
