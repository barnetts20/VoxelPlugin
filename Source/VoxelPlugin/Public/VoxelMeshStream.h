// VoxelMeshStream.h - A RealtimeMesh stream set for one brick's mesh, built on a worker and
// applied on the game thread (the CPU brick renderer and the collision bricks).

#pragma once

#include "CoreMinimal.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>

// The RealtimeMesh stream and key types used with this struct throughout the brick code.
using namespace RealtimeMesh;

/** Position, tangents, triangles, polygroups, texture coordinates and colour streams, with the
 *  buffer-set and section keys the mesh is applied under. */
struct VOXELPLUGIN_API FMeshStreamData
{
    FRealtimeMeshBufferSetKey MeshGroupKey;
    FRealtimeMeshSectionKey MeshSectionKey;
    FRealtimeMeshStreamSet MeshStream;

    FMeshStreamData()
    {
        MeshStream.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>());
        MeshStream.AddStream(FRealtimeMeshStreams::Tangents, GetRealtimeMeshBufferLayout<FRealtimeMeshTangentsNormalPrecision>());
        MeshStream.AddStream(FRealtimeMeshStreams::Triangles, GetRealtimeMeshBufferLayout<TIndex3<uint32>>());
        MeshStream.AddStream(FRealtimeMeshStreams::PolyGroups, GetRealtimeMeshBufferLayout<uint16>());
        MeshStream.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<FVector2DHalf>());
        MeshStream.AddStream(FRealtimeMeshStreams::Color, GetRealtimeMeshBufferLayout<FColor>());
    }

    TRealtimeMeshStreamBuilder<FVector, FVector3f> GetPositionStream()
    {
        return TRealtimeMeshStreamBuilder<FVector, FVector3f>(*MeshStream.Find(FRealtimeMeshStreams::Position));
    }
    TRealtimeMeshStreamBuilder<FRealtimeMeshTangentsHighPrecision, FRealtimeMeshTangentsNormalPrecision> GetTangentStream()
    {
        return TRealtimeMeshStreamBuilder<FRealtimeMeshTangentsHighPrecision, FRealtimeMeshTangentsNormalPrecision>(*MeshStream.Find(FRealtimeMeshStreams::Tangents));
    }
    TRealtimeMeshStreamBuilder<TIndex3<uint32>> GetTriangleStream()
    {
        return TRealtimeMeshStreamBuilder<TIndex3<uint32>>(*MeshStream.Find(FRealtimeMeshStreams::Triangles));
    }
    TRealtimeMeshStreamBuilder<uint32, uint16> GetPolygroupStream()
    {
        return TRealtimeMeshStreamBuilder<uint32, uint16>(*MeshStream.Find(FRealtimeMeshStreams::PolyGroups));
    }
    TRealtimeMeshStreamBuilder<FVector2f, FVector2DHalf> GetTexCoordStream()
    {
        return TRealtimeMeshStreamBuilder<FVector2f, FVector2DHalf>(*MeshStream.Find(FRealtimeMeshStreams::TexCoords));
    }
    TRealtimeMeshStreamBuilder<FColor> GetColorStream()
    {
        return TRealtimeMeshStreamBuilder<FColor>(*MeshStream.Find(FRealtimeMeshStreams::Color));
    }

    int32 NumTriangles()
    {
        const FRealtimeMeshStream* Triangles = MeshStream.Find(FRealtimeMeshStreams::Triangles);
        return Triangles ? Triangles->Num() : 0;
    }
};
