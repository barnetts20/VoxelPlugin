#include "FDensitySampleCompositor.h"
#include "FastNoise/FastNoise.h"


//TFunction<void(const FSampleInput&, float*)> FCompositorExamples::GetHeightMapExample(double ActorPlanetRadius, double ActorNoiseAmplitude, FastNoise::SmartNode<> InNoise)
//{
//    auto HeightmapLayer = [NoiseNode = InNoise, PlanetRadius = ActorPlanetRadius, NoiseAmplitude = ActorNoiseAmplitude](const FSampleInput& Input, float* DensityOut) {
//        int32 Count = Input.Num();
//
//        double InvNoiseAmplitude = 1.0 / NoiseAmplitude;
//        double RootExtent = (PlanetRadius + NoiseAmplitude) * 1.05;
//
//        // Stack buffers for the common small-count paths (8, 19 samples).
//        // Only heap-allocate for large bulk calls (ReconstructSubtree).
//        constexpr int32 StackLimit = 64;
//
//        float  StackPX[StackLimit], StackPY[StackLimit], StackPZ[StackLimit], StackNoise[StackLimit];
//        double StackDist[StackLimit];
//
//        TArray<float>  HeapPX, HeapPY, HeapPZ, HeapNoise;
//        TArray<double> HeapDist;
//
//        float* PX; float* PY; float* PZ; float* NoiseOut;
//        double* Distances;
//
//        if (Count <= StackLimit)
//        {
//            PX = StackPX; PY = StackPY; PZ = StackPZ; NoiseOut = StackNoise;
//            Distances = StackDist;
//        }
//        else
//        {
//            HeapPX.SetNumUninitialized(Count);
//            HeapPY.SetNumUninitialized(Count);
//            HeapPZ.SetNumUninitialized(Count);
//            HeapNoise.SetNumUninitialized(Count);
//            HeapDist.SetNumUninitialized(Count);
//            PX = HeapPX.GetData(); PY = HeapPY.GetData(); PZ = HeapPZ.GetData();
//            NoiseOut = HeapNoise.GetData(); Distances = HeapDist.GetData();
//        }
//
//        // Project positions onto sphere surface for noise sampling.
//        // Cache Dist to avoid recomputing sqrt in the density loop.
//        for (int32 i = 0; i < Count; i++)
//        {
//            double px = Input.X[i], py = Input.Y[i], pz = Input.Z[i];
//            double Dist = FMath::Sqrt(px * px + py * py + pz * pz);
//            Distances[i] = Dist;
//            double InvDist = (Dist > 1e-10) ? (RootExtent / Dist) : 0.0;
//            PX[i] = (float)(px * InvDist * InvNoiseAmplitude);
//            PY[i] = (float)(py * InvDist * InvNoiseAmplitude);
//            PZ[i] = (float)(pz * InvDist * InvNoiseAmplitude);
//        }
//
//        NoiseNode->GenPositionArray3D(NoiseOut, Count, PX, PY, PZ, 0, 0, 0, 0);
//
//        for (int32 i = 0; i < Count; i++)
//        {
//            double Clamped = FMath::Clamp((double)NoiseOut[i], -1.0, 1.0);
//            double Height = (Clamped + 1.0) * 0.5 * NoiseAmplitude;
//            DensityOut[i] = (float)(Distances[i] - (PlanetRadius + Height));
//        }
//        };
//    return HeightmapLayer;
//}
