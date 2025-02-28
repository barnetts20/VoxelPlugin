#include "FVoxelStructure.h"

FSamplePosition::FSamplePosition(int InSampleIndex, FVector InPosition, FVector InNormal, double InDensity)
{
    SampleIndex = InSampleIndex;
    Position = InPosition;
    Normal = InNormal;
    Density = InDensity;
}

FVoxelEdge::FVoxelEdge(int InEdgeIndex, FSamplePosition InEndPoints[2])
{
    EdgeIndex = InEdgeIndex;
    SignChange = false;

    // Copy the sample positions
    EndPoints[0] = InEndPoints[0];
    EndPoints[1] = InEndPoints[1];

    Length = FVector::Dist(EndPoints[0].Position, EndPoints[1].Position);
    MidPoint = (EndPoints[0].Position + EndPoints[1].Position) * 0.5f;
    SignChange = (EndPoints[0].Density < 0) != (EndPoints[1].Density < 0);
    // Calculate interpolation factor based on density values
    double t = FMath::Abs(EndPoints[0].Density) / (FMath::Abs(EndPoints[0].Density) + FMath::Abs(EndPoints[1].Density));
    ZeroCrossingPoint = (SignChange ? FMath::Lerp(EndPoints[0].Position, EndPoints[1].Position, t) : MidPoint);
    FVector InterpolatedNormal = FMath::Lerp(EndPoints[0].Normal, EndPoints[1].Normal, t).GetSafeNormal();
}

FQuadFace::FQuadFace(int InFaceIndex, FVoxelEdge InEdges[4])
{
    FaceIndex = InFaceIndex;
    MidPoint = FVector::ZeroVector;
    for (int i = 0; i < 4; i++) {
        Edges[i] = InEdges[i];
        MidPoint += Edges[i].MidPoint;
        Corners[i] = Edges[i].EndPoints[0];
    }
    MidPoint *= .25;
}

FTriFace::FTriFace(int InFaceIndex, FVoxelEdge InEdges[3])
{
    FaceIndex = InFaceIndex;
    MidPoint = FVector::ZeroVector;

    for (int i = 0; i < 3; i++) {
        Edges[i] = InEdges[i];
        MidPoint += Edges[i].MidPoint;
        Corners[i] = Edges[i].EndPoints[0];
    }
    MidPoint /= 3.0;
}

FTetrahedron::FTetrahedron(int InTetraIndex, FQuadFace InBaseFace, FSamplePosition InApex)
{
    TetraIndex = InTetraIndex;
    BaseFace = InBaseFace;
    Apex = InApex;

    FSamplePosition EdgeEnds[2];

    for (int i = 0; i < 4; ++i)
    {
        EdgeEnds[0] = BaseFace.Corners[i];
        EdgeEnds[1] = Apex;
        ConnectingEdges[i] = FVoxelEdge(12 + i, EdgeEnds); //Start edge idices after the initial 12 edges of the node boundaries, so 12 - 16
    }
    FVoxelEdge TriFaceEdges[3];
    for (int i = 0; i < 4; i++) {
        TriFaceEdges[0] = BaseFace.Edges[i];
        TriFaceEdges[1] = ConnectingEdges[i];
        TriFaceEdges[2] = ConnectingEdges[(i + 1) % 4];
        TriFaces[i] = FTriFace(6 + i, TriFaceEdges); //Start face index after the initial 6 faces of the node, so 6-10
    }

}