// Stand-in for UE's ParallelFor (the harness runs it in order).
#pragma once
#include "../CoreMinimal.h"
template<class F> void ParallelFor(int32 Num, F&& Body) { for (int32 i = 0; i < Num; i++) Body(i); }
