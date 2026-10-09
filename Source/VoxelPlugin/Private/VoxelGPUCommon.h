// VoxelGPUCommon.h - Render-graph helpers shared by the density and mesher GPU code (private
// to the module: it pulls in the render graph headers).

#pragma once

#include "CoreMinimal.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderingThread.h"
#include "Containers/Ticker.h"
#include <atomic>

namespace VoxelGPUCommon
{
    /** Render thread. Uploads Data as a structured buffer of T. A structured buffer can't be
     *  empty, so no data gives one zeroed element. */
    template<typename T>
    FRDGBufferRef UploadStructuredBuffer(FRDGBuilder& GraphBuilder, const TCHAR* Name, TConstArrayView<T> Data)
    {
        static const T Zero{};
        if (Data.Num() == 0)
            return CreateStructuredBuffer(GraphBuilder, Name, sizeof(T), 1, &Zero, sizeof(T));
        return CreateStructuredBuffer(GraphBuilder, Name, sizeof(T), Data.Num(), Data.GetData(), sizeof(T) * Data.Num());
    }

    /** Runs Step on the render thread once per frame until it returns true (readback
     *  polling). Step is shared between frames, so it must own what it touches. */
    inline void PollRenderThread(TFunction<bool()> Step)
    {
        struct FPoll
        {
            TFunction<bool()> Step;
            std::atomic<bool> bDone{ false };
        };
        const TSharedRef<FPoll, ESPMode::ThreadSafe> Poll = MakeShared<FPoll, ESPMode::ThreadSafe>();
        Poll->Step = MoveTemp(Step);
        FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([Poll](float) -> bool
            {
                if (Poll->bDone.load(std::memory_order_acquire)) return false;
                ENQUEUE_RENDER_COMMAND(VoxelGPUPoll)([Poll](FRHICommandListImmediate&)
                    {
                        if (Poll->bDone.load(std::memory_order_acquire)) return;
                        if (Poll->Step())
                            Poll->bDone.store(true, std::memory_order_release);
                    });
                return true;
            }));
    }
}
