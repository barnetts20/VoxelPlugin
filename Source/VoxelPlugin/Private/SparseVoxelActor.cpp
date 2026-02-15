// Fill out your copyright notice in the Description page of Project Settings.


#include "SparseVoxelActor.h"
#include "Kismet/KismetSystemLibrary.h"
#include <Kismet/GameplayStatics.h>

// Sets default values
ASparseVoxelActor::ASparseVoxelActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    bDebugNeedsRedraw = false;
}

// Called when the game starts or when spawned
void ASparseVoxelActor::BeginPlay()
{
	Super::BeginPlay();
	InitializeOctree();
	
}

// Called every frame
void ASparseVoxelActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
    //UpdateNodes();
    auto world = GetWorld();
    if (world != nullptr) {
        auto viewLocations = world->ViewLocationsRenderedLastFrame;
        if (viewLocations.Num() > 0) {
            this->CameraPosition = viewLocations[0];
        }
    }

	if (bShowDebug && bDebugNeedsRedraw)
	{
        DebugDrawOctree();
	}
}
void ASparseVoxelActor::InitializeOctree()
{
	VoxelOctree = MakeShared<FSparseOctree>();
    TSharedPtr<FVoxelData> insertData = MakeShared<FVoxelData>();
    insertData->Density = 1;
    insertData->ObjectId = 1;
    //VoxelOctree->PopulateSphereInOctree(FVector(0, 0, 0), 36, 400.0, insertData);
    VoxelOctree->PopulatePointsInOctree();
    bDebugNeedsRedraw = true;
}

void ASparseVoxelActor::DebugDrawOctree()
{
    if (!VoxelOctree.IsValid()) return;
    FlushPersistentDebugLines(GetWorld());
    TArray<TSharedPtr<FSparseOctreeNode>> NodesToDraw;
    NodesToDraw.Add(VoxelOctree->Root);

    while (NodesToDraw.Num() > 0)
    {
        TSharedPtr<FSparseOctreeNode> Node = NodesToDraw.Pop();
        if (!Node.IsValid()) continue;
        if (Node->GetHalfScale() < Precision) continue;

        if (Node->HasPayload()) {
            DebugDrawNode(Node);
        }
        //**Push children onto the stack for processing**
        for (const TSharedPtr<FSparseOctreeNode>& Child : Node->GetChildren())
        {
            if (Child.IsValid())  // Only add valid children
            {
                NodesToDraw.Add(Child);
            }
        }
    }
}

void ASparseVoxelActor::DebugDrawNode(TSharedPtr<FSparseOctreeNode> node) {
    FInt64Coordinate coordToUse = node->GetCenter();
    FColor DebugColor = FColor::Green;
    if (node->HasPayload()) {
        TSharedPtr<FDCVoxelData> DCVoxelData = StaticCastSharedPtr<FDCVoxelData>(node->GetPayload());

        if (DCVoxelData.IsValid())
        {
            coordToUse = DCVoxelData->DCPosition; // Use computed DC vertex
        }
        if (node->GetPayload()->Density > 0) {
            DebugColor = FColor::Red;
        }
    }
    else {
        DebugColor = FColor::Blue;
    }

    FVector Center = VoxelOctree->ConvertToWorldPosition(coordToUse);
    float Size = static_cast<float>(node->GetHalfScale()) * VoxelOctree->GetPrecision();


    //DrawDebugBox(GetWorld(), VoxelOctree->ConvertToWorldPosition(node->GetCenter()), FVector(Size), FColor::Blue, false);
    DrawDebugPoint(GetWorld(), Center, 2, DebugColor);
}
