/**
 * @author Felix Thielke
 */

#include "CompilationSettings.h"
#include <asmjit/asmjit.h>

using namespace asmjit;

void NeuralNetwork::CompilationSettings::constrict()
{
  const CpuInfo& cpuInfo = CpuInfo::getHost();

  if(useX64 && cpuInfo.getArchType() != ArchInfo::Type::kTypeX64)
    useX64 = false;

  if(useSSE42 && !cpuInfo.hasFeature(CpuInfo::X86Features::kX86FeatureSSE4_2))
    useSSE42 = false;

  if(useAVX2 && !cpuInfo.hasFeature(CpuInfo::X86Features::kX86FeatureAVX2))
    useAVX2 = false;

  Rangeuc(1, 8).clamp(quantizationResolution);
}
