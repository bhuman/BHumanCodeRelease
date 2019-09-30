/**
 * @author Felix Thielke
 */

#include "CompilationSettings.h"
#include <asmjit/asmjit.h>

using namespace asmjit;

void NeuralNetwork::CompilationSettings::constrict()
{
  const CpuInfo& cpuInfo = CpuInfo::host();

  if(useX64 && !ArchInfo(cpuInfo.archId(), cpuInfo.archSubId()).is64Bit())
    useX64 = false;

  if(useSSE42 && !cpuInfo.features<x86::Features>().hasSSE4_2())
    useSSE42 = false;

  if(useAVX2 && !cpuInfo.features<x86::Features>().hasAVX2())
    useAVX2 = false;
}
