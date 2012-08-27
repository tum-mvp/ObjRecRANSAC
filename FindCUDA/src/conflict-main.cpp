
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>

#include "external_dependency.h"

extern "C" int doit_squared();
extern "C" int doit_cubed();

int main( int argc, char **argv )
{
  CHECK_CUDA_ERROR();
  int result1 = doit_squared();
  int result2 = doit_cubed();
  return result1 + result2;
};

