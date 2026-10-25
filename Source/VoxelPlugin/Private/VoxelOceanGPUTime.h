#pragma once

#include "CoreMinimal.h"
#include "Templates/SharedPointer.h"

class FRDGBuilder;

/**
 * Stage W8f: the ocean's own GPU time, pass group by pass group.
 *
 * The performance sweep timed the whole GPU frame, whose baseline swung by 3 ms between segments
 * with nothing changed (the editor, the other viewports, the GPU's clocks): a feature under 2 ms
 * could not be told from the noise. Each group of the ocean's passes is now bracketed by a pair of
 * GPU timestamps (RQT_AbsoluteTime queries, an RDG pass each that never culls), read back without
 * waiting a few frames later and summed per frame and group.
 *
 * Off (the default) the scopes add nothing to the graph. The sweep turns it on while it runs.
 * A group's spans in a frame are summed (every view, every viewport); a frame with no span of a
 * group reads 0 for it. The time is the GPU's wall time between the pair: what the GPU spends
 * inside the group, including its own stalls and barriers.
 */
namespace VoxelOceanGPUTime
{
    enum EGroup : int32
    {
        Cascades = 0,   // the FFT cascades (VoxelOceanCascades)
        Clipmap,        // the clipmap's static, waves and fine foam layers (VoxelOceanClipmap)
        Mesh,           // the displaced tiles and the rings (VoxelOceanDisplace, VoxelOceanRings)
        Surface,        // the surface's draw (VoxelOceanSurface)
        Spray,          // the spray's update, spawn and draw
        Underwater,     // the underwater view and its camera probe (VoxelUnderwater)
        Other,          // the page writes, the depth probes
        Behind,         // stage W8h: the copy of what lies under the water (VoxelOceanBehind)
        /** Stage W8g: a fixed workload (VoxelOceanGPUTimeRef.usf), not the ocean's: the GPU's
         *  speed, which the sweep scales the ocean's times by. Not in FFrameTimes::Total. */
        Reference,
        NumGroups
    };
    /** The ocean's own groups: those before Reference. */
    constexpr int32 NumOceanGroups = (int32)Reference;

    const TCHAR* GroupName(int32 Group);

    /** One frame's resolved times (ms). */
    struct FFrameTimes
    {
        uint32 Frame = 0;
        float Ms[NumGroups] = {};
        float Total = 0.0f;         // the ocean's groups (not the reference)
    };

    /** Any thread: on or off. Off, the queries are let go on the render thread. */
    void SetEnabled(bool bOn);
    bool IsEnabled();

    /** Any thread: the frames resolved since the last call, oldest first (at most 512 kept). */
    void Drain(TArray<FFrameTimes>& Out);

    /** Stage W8g, graph setup: the reference workload, timed as Reference (nothing when off). */
    void AddReference(FRDGBuilder& GraphBuilder);

    /** One span of a group (render thread). */
    struct FSpan;

    /** Graph setup (any thread, behind a lock): a timestamp opening a span of Group here in the graph (null when off). */
    TSharedPtr<FSpan> Begin(FRDGBuilder& GraphBuilder, int32 Group);
    /** Graph setup: the span's closing timestamp. */
    void End(FRDGBuilder& GraphBuilder, const TSharedPtr<FSpan>& Span);

    /** A span over a C++ scope (closed early with End()). */
    class FScope
    {
    public:
        FScope(FRDGBuilder& InGraphBuilder, int32 Group)
            : GraphBuilder(InGraphBuilder), Span(Begin(InGraphBuilder, Group))
        {
        }
        ~FScope() { End(); }
        void End()
        {
            if (Span.IsValid())
            {
                VoxelOceanGPUTime::End(GraphBuilder, Span);
                Span.Reset();
            }
        }
        FScope(const FScope&) = delete;
        FScope& operator=(const FScope&) = delete;

    private:
        FRDGBuilder& GraphBuilder;
        TSharedPtr<FSpan> Span;
    };
}
