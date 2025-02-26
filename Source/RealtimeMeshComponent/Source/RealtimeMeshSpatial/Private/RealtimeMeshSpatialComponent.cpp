// Copyright TriAxis Games, L.L.C. All Rights Reserved.


#include "RealtimeMeshSpatialComponent.h"

#include "RealtimeMeshComponent.h"
#include "RealtimeMeshSimple.h"
#include "RealtimeMeshSpatialComponentLocation.h"
#include "RealtimeMeshSpatialStreamingCellState.h"
#include "Mesh/RealtimeMeshBasicShapeTools.h"
#include "RealtimeMeshSpatialStreamingSubsystem.h"
#include "Algo/AllOf.h"
#include "Algo/AnyOf.h"
#include "Algo/RemoveIf.h"
#include "Spatial/RealtimeMeshSpatialValidChunkProvider.h"

using namespace RealtimeMesh;

static constexpr float StreamingScalar = 1.25;


DECLARE_CYCLE_STAT(TEXT("RealtimeMeshSpatial - GetWantedStreamingState"), STAT_RealtimeMesh_SpatialGetWantedStreamingState, STATGROUP_RealtimeMesh);
DECLARE_CYCLE_STAT(TEXT("RealtimeMeshSpatial - UpdateStreaming"), STAT_RealtimeMesh_SpatialUpdateStreaming, STATGROUP_RealtimeMesh);
DECLARE_CYCLE_STAT(TEXT("RealtimeMeshSpatial - CalculateDesiredTree"), STAT_RealtimeMesh_CalculatedDesiredTree, STATGROUP_RealtimeMesh);


URealtimeMeshSpatialComponent::URealtimeMeshSpatialComponent()
{
}

void URealtimeMeshSpatialComponent::Initialize(const TSharedRef<RealtimeMesh::IRealtimeMeshSpatialStreamingStructureProvider>& InChunkProvider)
{
	Reset();
	ChunkProviderWeak = InChunkProvider;
	InChunkProvider->OnCellChanged().BindUObject(this, &URealtimeMeshSpatialComponent::OnCellChanged);
}

void URealtimeMeshSpatialComponent::Reset()
{
	for (auto& Cell : ActiveCells)
	{
		Cell.Value.PendingCancelToken.Cancel();
		if (IsValid(Cell.Value.MeshComponent))
		{
			Cell.Value.MeshComponent->DestroyComponent();
		}
	}
	ActiveCells.Empty();
	RecycledMeshes.Empty();

	if (auto ChunkProviderPinned = ChunkProviderWeak.Pin())
	{
		ensureMsgf(ChunkProviderPinned->OnCellChanged().IsBoundToObject(this), TEXT("Provider not bound to this component"));
		ChunkProviderPinned->OnCellChanged().Unbind();
	}
	
	ChunkProviderWeak.Reset();
}

void URealtimeMeshSpatialComponent::UpdateStreaming(const TArray<FRealtimeMeshSpatialStreamingSource>& Sources)
{
	SCOPE_CYCLE_COUNTER(STAT_RealtimeMesh_SpatialUpdateStreaming);
	
	// If we have a streaming policy, update the streaming.
	if (auto ChunkProvider = ChunkProviderWeak.Pin())
	{		
		TSet<FRealtimeMeshSpatialComponentLocation> CellsToActivate, CellsToDeactivate;
		TSet<FRealtimeMeshSpatialComponentLocation> CellsToLoad, CellsToUnload;
		
		GetWantedStreamingState(Sources, ActiveCells, CellsToDeactivate, CellsToUnload, CellsToActivate, CellsToLoad);

		//TSet<FRealtimeMeshSpatialComponentLocation> CellsTouched = CellsToDeactivate.Union(CellsToActivate).Union(CellsToLoad).Union(CellsToUnload);
		// This check is to make sure that we are not doing multiple operations to the same cell in the same frame.
		//check(CellsTouched.Num() == CellsToDeactivate.Num() + CellsToUnload.Num() + CellsToActivate.Num() + CellsToLoad.Num());

		/*check(CellsToActivate.Intersect(CellsToDeactivate).Num() == 0);
		check(CellsToActivate.Intersect(CellsToLoad).Num() == 0);
		check(CellsToActivate.Intersect(CellsToUnload).Num() == 0);
		check(CellsToDeactivate.Intersect(CellsToLoad).Num() == 0);
		check(CellsToDeactivate.Intersect(CellsToUnload).Num() == 0);
		check(CellsToLoad.Intersect(CellsToUnload).Num() == 0);*/

		

		// Handle deactivation of cells
		for (const auto& Cell : CellsToDeactivate)
		{
			auto& CellState = ActiveCells.FindChecked(Cell);			
			check(CellState.IsVisible());
			if (IsValid(CellState.MeshComponent))
			{
				DeactivateCell(Cell, CellState.MeshComponent);
			}
			CellState.bShouldBeVisible = false;
		}
		
		// Handle activation of cells
		for (const auto& Cell : CellsToActivate)
		{
			auto& CellState = ActiveCells.FindChecked(Cell);	
			check(CellState.bIsInitialized && !CellState.bShouldBeVisible);
			if (IsValid(CellState.MeshComponent))
			{
				ActivateCell(Cell, CellState.MeshComponent);
			}
			CellState.bShouldBeVisible = true;
		}

		// Handle unload of cells
		for (const auto& Cell : CellsToUnload)
		{
			auto& CellState = ActiveCells.FindChecked(Cell);

			// Set cancel token to stop any pending generation
			CellState.PendingCancelToken.Cancel();

			if (IsValid(CellState.MeshComponent))
			{
				if (CellState.bShouldBeVisible)
				{
					DeactivateCell(Cell, CellState.MeshComponent);
				}
				UnloadCell(Cell, CellState.MeshComponent->GetRealtimeMesh());
				RecycleMesh(CellState.MeshComponent);
			}
			ActiveCells.Remove(Cell);
		}

		// Handle load of cells
		for (const auto& Cell : CellsToLoad)
		{
			check(!ActiveCells.Contains(Cell));

			auto& CellState = ActiveCells.Add(Cell);
			CellState.bIsLoading = true;
			CellState.PendingFuture = ContinueOnGameThread(LoadCell(Cell, CellState.PendingCancelToken), [this, Cell, SerialKey = CellState.SerialKey](TFuture<URealtimeMesh*>&& MeshFuture)
			{
				FinalizeCellLoad(Cell, MeshFuture.Get(), false, INDEX_NONE, SerialKey);
			});
		}





		if (false)
		{
			static FColor LODColors[] =
			{
				FColor::Red,
				FColor::Blue,
				FColor::Green,
				FColor::Orange,
				FColor::Purple
			};
			
			for (const auto& ActiveCell : ActiveCells)
			{
				if (ActiveCell.Value.bShouldBeVisible)
				{
					const auto& Loc = ActiveCell.Key;

					FVector3d CellSize = ChunkProvider->GetCellSize(Loc.LOD);
					const FVector3d Extents = CellSize / 2.0f;
					FVector3d CellPos = ChunkProvider->GetCellLocation(Loc) * GetComponentScale();
				

					::DrawDebugBox(GetWorld(),  GetComponentRotation().RotateVector(CellPos) + GetComponentLocation(),
						Extents * GetComponentScale(), GetComponentRotation().Quaternion(), LODColors[Loc.LOD], false,
						0.01, SDPG_World, 5.0 * FMath::Pow(2.0f, Loc.LOD));
				}
			}
		}
	}
	else
	{
		// Unload all existing cell since we have no streaming policy
		for (auto It = ActiveCells.CreateIterator(); It; ++It)
		{
			auto& CellState = It.Value();
			CellState.PendingCancelToken.Cancel();
			if (IsValid(CellState.MeshComponent))
			{
				if (CellState.bShouldBeVisible)
				{
					DeactivateCell(It.Key(), CellState.MeshComponent);
				}
				UnloadCell(It.Key(), CellState.MeshComponent->GetRealtimeMesh());
				RecycleMesh(CellState.MeshComponent);
			}

			// Remove this cell
			It.RemoveCurrent();
		}

		// Destroy recycled meshes to free memory
		for (auto& Mesh : RecycledMeshes)
		{
			Mesh->DestroyComponent();
		}
		RecycledMeshes.Empty();
	}
}

// Called when the game starts or when spawned
void URealtimeMeshSpatialComponent::BeginPlay()
{
	Super::BeginPlay();
	
}

void URealtimeMeshSpatialComponent::OnRegister()
{
	Super::OnRegister();

	if (const UWorld* World = GetWorld())
	{
		if (auto* RealtimeMeshSpatialStreamingSubsystem = World->GetSubsystem<URealtimeMeshSpatialStreamingSubsystem>())
		{
			RealtimeMeshSpatialStreamingSubsystem->RegisterActor(this);
		}
	}
}

void URealtimeMeshSpatialComponent::OnUnregister()
{
	if (const UWorld* World = GetWorld())
	{
		if (auto* RealtimeMeshSpatialStreamingSubsystem = World->GetSubsystem<URealtimeMeshSpatialStreamingSubsystem>())
		{
			RealtimeMeshSpatialStreamingSubsystem->UnregisterActor(this);
		}
	}

	Super::OnUnregister();
}

#if WITH_EDITOR
void URealtimeMeshSpatialComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	const FName PropertyName = (PropertyChangedEvent.Property != nullptr) ? PropertyChangedEvent.Property->GetFName() : NAME_None;
	
	if (PropertyName == GET_MEMBER_NAME_CHECKED(URealtimeMeshSpatialComponent, Materials))
	{
		for (const auto& Cell : ActiveCells)
		{
			if (IsValid(Cell.Value.MeshComponent))
			{
				Cell.Value.MeshComponent->OverrideMaterials = Materials;
			}
		}
	}
	
	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

FSphere3d URealtimeMeshSpatialComponent::GetStreamingSphereForSourceAndLOD(const FRealtimeMeshSpatialStreamingSource& StreamingSource, int32 LOD)
{
	constexpr float LODMultiplierBase = 2.5f;
	constexpr float LODMultiplierExp = 1.0f;

	const double MaxStreamingRadius = StreamingSource.Radius * FMath::Pow(LODMultiplierBase, LODMultiplierExp * LOD);

	return FSphere3d(FVector3d(StreamingSource.Location.X, StreamingSource.Location.Y, StreamingSource.Location.Z), MaxStreamingRadius);
}

TArray<TArray<FSphere3d>> URealtimeMeshSpatialComponent::GetStreamingSpheres(const TArrayView<const FRealtimeMeshSpatialStreamingSource> StreamingSources, float ScaleFactor)
{
	const auto ChunkProvider = ChunkProviderWeak.Pin();
	check(ChunkProvider);
	
	TArray<TArray<FSphere3d>> LODStreamingSpheres;
	LODStreamingSpheres.SetNum(ChunkProvider->GetMaxLOD() + 1);

	for (int32 Index = 0; Index <= ChunkProvider->GetMaxLOD(); Index++)
	{
		for (const auto& StreamingSource : StreamingSources)
		{
			const FSphere3d Sphere = GetStreamingSphereForSourceAndLOD(StreamingSource, Index); 
			LODStreamingSpheres[Index].Add(FSphere3d(Sphere.Center, Sphere.W * ScaleFactor));
		}
	}

	return LODStreamingSpheres;
}

TSet<FRealtimeMeshSpatialComponentLocation> URealtimeMeshSpatialComponent::CalculatedDesiredTree(const TArrayView<const FRealtimeMeshSpatialStreamingSource> StreamingSources,
                                                                                                 float ScaleFactor, const TFunctionRef<bool(const FRealtimeMeshSpatialComponentLocation&)>& IsReadyFunc)
{
	SCOPE_CYCLE_COUNTER(STAT_RealtimeMesh_CalculatedDesiredTree);
	
	const auto ChunkProvider = ChunkProviderWeak.Pin();
	check(ChunkProvider);
	
	const TArray<TArray<FSphere3d>> LODStreamingSpheres = GetStreamingSpheres(StreamingSources, 1.0f);
	const FVector3d MaxCellSize = ChunkProvider->GetCellSize(ChunkProvider->GetMaxLOD());

	TSet<FRealtimeMeshSpatialComponentLocation> CellTree;
	CellTree.Reserve(128 * 1024);

	// Walk all the starter cells and recursively decide whether to subdivide.

	TFunction<bool(const FRealtimeMeshSpatialComponentLocation&)> SubdivideIfPossible = [&](const FRealtimeMeshSpatialComponentLocation& ParentCell) -> bool
	{
		TArray<FInt64Vector, TFixedAllocator<8>> ChildCells =
		{
			FInt64Vector(0, 0, 0),
			FInt64Vector(0, 0, 1),
			FInt64Vector(0, 1, 0),
			FInt64Vector(0, 1, 1),
			FInt64Vector(1, 0, 0),
			FInt64Vector(1, 0, 1),
			FInt64Vector(1, 1, 0),
			FInt64Vector(1, 1, 1)
		};
		const int32 ChildLODIndex = ParentCell.LOD - 1;
		const FVector3d ChildCellSize = ChunkProvider->GetCellSize(ChildLODIndex);

		// Remove invalid children
		ChildCells.SetNum(Algo::RemoveIf(ChildCells, [&](const FInt64Vector& Child)
		{
			return !ChunkProvider->IsCellValid(Child, ChildLODIndex);
		}));

		// Test if all of the children are ready to be subdivided
		bool bAllCanSubdivide = Algo::AllOf(ChildCells, [&](const FInt64Vector& Child)
		{
			return IsReadyFunc(FRealtimeMeshSpatialComponentLocation(Child, ChildLODIndex));
		});

		// Not all children are ready, so we can't subdivide
		if (!bAllCanSubdivide)
		{
			return false;
		}

		// Test if any of the children intersect the streaming radius
		bool bDoesAnyIntersect = Algo::AnyOf(ChildCells, [&](const FInt64Vector& Child)
		{
			const FBox ChildBox(FVector3d(Child.X, Child.Y, Child.Z) * ChildCellSize, FVector3d(Child.X, Child.Y, Child.Z) * ChildCellSize + ChildCellSize);
			return Algo::AnyOf(LODStreamingSpheres[ChildLODIndex], [&](const FSphere3d& StreamingSource)
			{
				return FMath::SphereAABBIntersection(StreamingSource, ChildBox);
			});
		});

		// None of the children intersect this lods streaming radius
		if (!bDoesAnyIntersect)
		{
			return false;
		}

		// Test if we should subdivide the children? if not add the children to the list
		for (const FInt64Vector& Child : ChildCells)
		{
			FRealtimeMeshSpatialComponentLocation ChildLocation(Child, ChildLODIndex);
		
			// Should we subdivide this child?
			if (ChildLocation.LOD > 0 && SubdivideIfPossible(ChildLocation))
			{
				// We subdivided the child, so we don't need to add it to the list
				continue;
			}

			// Since we didn't subdivide, add this node
			CellTree.Add(ChildLocation);
		}
	
		return true;
	};

	// find the starting set of chunks at max LOD
	for (const FSphere3d& Sphere : LODStreamingSpheres[ChunkProvider->GetMaxLOD()])
	{
		// find min/max bounds of the sphere in cells
		FIntVector GridPosMin = FIntVector(
			FMath::FloorToInt((Sphere.Center.X - Sphere.W) / MaxCellSize.X),
			FMath::FloorToInt((Sphere.Center.Y - Sphere.W) / MaxCellSize.Y),
			FMath::FloorToInt((Sphere.Center.Z - Sphere.W) / MaxCellSize.Z));
		FIntVector GridPosMax = FIntVector(
			FMath::CeilToInt((Sphere.Center.X + Sphere.W) / MaxCellSize.X),
			FMath::CeilToInt((Sphere.Center.Y + Sphere.W) / MaxCellSize.Y),
			FMath::CeilToInt((Sphere.Center.Z + Sphere.W) / MaxCellSize.Z));

		// Loop through all cells and if they intersect the sphere add it to the starter list
		for (int32 X = GridPosMin.X; X <= GridPosMax.X; ++X)
		{
			for (int32 Y = GridPosMin.Y; Y <= GridPosMax.Y; ++Y)
			{
				for (int32 Z = GridPosMin.Z; Z <= GridPosMax.Z; ++Z)
				{
					// If the cell isn't valid, skip it.
					if (!ChunkProvider->IsCellValid(FInt64Vector(X, Y, Z), ChunkProvider->GetMaxLOD()))
					{
						continue;
					}
					
					// Get the bounds of the cell
					const FBox3d CellBox(FVector3d(X, Y, Z) * MaxCellSize, FVector3d(X, Y, Z) * MaxCellSize + MaxCellSize);
					if (!FMath::SphereAABBIntersection(Sphere, CellBox))
					{
						continue;
					}
				
					FRealtimeMeshSpatialComponentLocation CellLocation(FInt64Vector(X, Y, Z), ChunkProvider->GetMaxLOD());
					if (CellLocation.LOD == 0 || !SubdivideIfPossible(CellLocation))
					{
						if (IsReadyFunc(CellLocation))
						{
							CellTree.Add(CellLocation);
						}
					}
				}
			}
		}
	}

	return CellTree;
}

void URealtimeMeshSpatialComponent::GetWantedStreamingState(const TArrayView<const FRealtimeMeshSpatialStreamingSource> StreamingSources,
	const TMap<FRealtimeMeshSpatialComponentLocation, FRealtimeMeshSpatialStreamingCellState>& ExistingCells, TSet<FRealtimeMeshSpatialComponentLocation>& CellsToDeActivate,
	TSet<FRealtimeMeshSpatialComponentLocation>& CellsToUnload, TSet<FRealtimeMeshSpatialComponentLocation>& CellsToActivate,
	TSet<FRealtimeMeshSpatialComponentLocation>& CellsToLoad)
{	
	SCOPE_CYCLE_COUNTER(STAT_RealtimeMesh_SpatialGetWantedStreamingState);
	
	if (StreamingSources.Num() <= 0)
	{
		// Nothing we can load here.
		return;
	}

	auto LoadTree = CalculatedDesiredTree(StreamingSources, StreamingScalar, [](const FRealtimeMeshSpatialComponentLocation& Cell)
	{
		return true;
	});
	
	auto ActivateTree = CalculatedDesiredTree(StreamingSources, 1.0f, [&ExistingCells](const FRealtimeMeshSpatialComponentLocation& Cell) -> bool
	{
		if (auto* ExistingCell = ExistingCells.Find(Cell))
		{
			return ExistingCell->bIsInitialized;
		}
		return false;
	});

	// All cells to activate should be in the load tree as well
	check(ActivateTree.Intersect(LoadTree).Num() == ActivateTree.Num());

	// All cells to activate should be existing and loaded
	check(Algo::AllOf(ActivateTree, [&ExistingCells](const FRealtimeMeshSpatialComponentLocation& Cell)
	{
		auto& ExistingCell = ExistingCells.FindChecked(Cell);
		return ExistingCell.bIsInitialized;
	}));

	// Grab all the existing keys
	TSet<FRealtimeMeshSpatialComponentLocation> ExistingCellsKeys;
	ExistingCells.GetKeys(ExistingCellsKeys);

	// Find cells to load
	Algo::CopyIf(LoadTree, CellsToLoad, [&ExistingCells](const FRealtimeMeshSpatialComponentLocation& Cell)
	{
		return !ExistingCells.Contains(Cell);
	});

	// Find cells to unload
	Algo::CopyIf(ExistingCellsKeys, CellsToUnload, [&LoadTree, &ExistingCells](const FRealtimeMeshSpatialComponentLocation& Cell)
	{
		return !ExistingCells[Cell].bShouldBeVisible && !LoadTree.Contains(Cell);
	});

	// Find cells to activate
	Algo::CopyIf(ActivateTree, CellsToActivate, [&ExistingCells](const FRealtimeMeshSpatialComponentLocation& Cell)
	{
		auto& ExistingCell = ExistingCells.FindChecked(Cell);
		check(ExistingCell.bIsInitialized);
		return !ExistingCell.bShouldBeVisible;
	});

	// Find cells to deactivate
	Algo::CopyIf(ExistingCellsKeys, CellsToDeActivate, [&ActivateTree, &ExistingCells](const FRealtimeMeshSpatialComponentLocation& Cell)
	{
		return ExistingCells[Cell].bShouldBeVisible && !ActivateTree.Contains(Cell);
	});
}

void URealtimeMeshSpatialComponent::ActivateCell(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMeshComponent* MeshComponent)
{
	MeshComponent->SetVisibility(true);
}

void URealtimeMeshSpatialComponent::DeactivateCell(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMeshComponent* MeshComponent)
{
	MeshComponent->SetVisibility(false);
}

TFuture<URealtimeMesh*> URealtimeMeshSpatialComponent::LoadCell(const FRealtimeMeshSpatialComponentLocation& Cell, RealtimeMesh::FRealtimeMeshCancellationToken CancellationToken)
{
	const auto ChunkProvider = ChunkProviderWeak.Pin();
	check(ChunkProvider);
	
	URealtimeMeshSimple* RMSimple = NewObject<URealtimeMeshSimple>();
	RMSimple->SetupMaterialSlot(0, "PrimaryMaterial");

	FRealtimeMeshStreamSet StreamSet;
	TRealtimeMeshBuilderLocal<uint16, FPackedNormal, FVector2DHalf, 2> Builder(StreamSet);
	Builder.EnableTangents();
	Builder.EnableTexCoords();
	Builder.EnableColors();
	Builder.EnablePolyGroups();

	FVector3d CellSize = ChunkProvider->GetCellSize(Cell.LOD);
	
	URealtimeMeshBasicShapeTools::AppendBoxMesh(StreamSet, FVector3f(CellSize.X / 2.0f, CellSize.Y / 2.0f, 25.0f),
		FTransform3f(FVector3f(CellSize.X / 2.0f, CellSize.Y / 2.0f, 0.0f)));
	

	const FRealtimeMeshSectionGroupKey GroupKey = FRealtimeMeshSectionGroupKey::Create(0, FName("TestBox"));
	const FRealtimeMeshSectionKey PolyGroup0SectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey, 0);
	RMSimple->CreateSectionGroup(GroupKey, StreamSet);
	RMSimple->UpdateSectionConfig(PolyGroup0SectionKey, FRealtimeMeshSectionConfig(0), true);

	return MakeFulfilledPromise<URealtimeMesh*>(RMSimple).GetFuture();	
}

TFuture<bool> URealtimeMeshSpatialComponent::UpdateCell(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMesh* Mesh, FRealtimeMeshCancellationToken CancellationToken)
{
	// This is to be implemented in derived types to handle updating a cell
	return MakeFulfilledPromise<bool>(true).GetFuture();
}

void URealtimeMeshSpatialComponent::UnloadCell(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMesh* Mesh)
{
	// We don't need to do anything here, as the mesh will be destroyed by the GC.
	// We could override this to recycle the mesh if we wanted to
}

void URealtimeMeshSpatialComponent::FinalizeCellLoad(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMesh* InMesh, bool bIsReload, int32 ChangeId, int32 SerialKey)
{
	const auto ChunkProvider = ChunkProviderWeak.Pin();
	if (!ChunkProvider)
	{
		return;
	}
	
	check(IsInGameThread());

	// Make sure this wasn't unloaded before being fully loaded
	if (!ActiveCells.Contains(Cell))
	{
		return;
	}
	
	auto& CellState = ActiveCells.FindChecked(Cell);

	if (CellState.SerialKey != SerialKey)
	{
		// This means it's a stale operation, so we can just ignore it.
		return;
	}
	
	check(CellState.bIsLoading && (bIsReload || !CellState.bIsInitialized));
	//check(CellState.MeshComponent == nullptr);
	
	// We always flag the cell as loaded, even if there's no valid mesh data
	if (!bIsReload)
	{
		CellState.bIsInitialized = !CellState.PendingCancelToken.IsCancelled();		
	}

	if (bIsReload && ChangeId < CellState.CompletedChangeVersion)
	{
		return;
	}
	CellState.bIsLoading = false;
	CellState.CompletedChangeVersion = ChangeId;

	if (!bIsReload || ChangeId == CellState.CompletedChangeVersion)
	{		
		CellState.PendingFuture.Reset();
		CellState.PendingCancelToken = FRealtimeMeshCancellationToken();
	}
	
	if (!IsValid(InMesh) || CellState.PendingCancelToken.IsCancelled())
	{
		CellState.MeshComponent = nullptr;
		return;
	}
	
	URealtimeMeshComponent* MeshComponent = nullptr;
	if (!RecycledMeshes.IsEmpty())
	{
		MeshComponent = RecycledMeshes.Pop();
		MeshComponent->Rename(*GetCellName(Cell));
	}
	else
	{
		MeshComponent = NewObject<URealtimeMeshComponent>(this, *GetCellName(Cell), GetCellComponentFlags());	
		MeshComponent->SetupAttachment(this);
		MeshComponent->RegisterComponent();	
	}

	MeshComponent->SetRelativeTransform(FTransform(ChunkProvider->GetCellLocation(Cell)));
	MeshComponent->SetVisibility(false);
	MeshComponent->OverrideMaterials = Materials;
	
	MeshComponent->SetRealtimeMesh(InMesh);
	
	CellState.MeshComponent = MeshComponent;

	// If we have a pending update, kick it off now.
	if (CellState.bWantsUpdate2)
	{
		OnCellChanged(Cell);
	}
}


void URealtimeMeshSpatialComponent::FinalizeCellUpdate(const FRealtimeMeshSpatialComponentLocation& Cell, int32 ChangeId, bool bHasMesh, int32 SerialKey)
{
	const auto ChunkProvider = ChunkProviderWeak.Pin();
	check(ChunkProvider);
	
	check(IsInGameThread());

	// Make sure this wasn't unloaded before being fully loaded
	if (!ActiveCells.Contains(Cell))
	{
		return;
	}
		
	auto& CellState = ActiveCells.FindChecked(Cell);

	if (CellState.SerialKey != SerialKey)
	{
		// This means it's a stale operation, so we can just ignore it.
		return;
	}
	
	// If we are still the latest pending, clear the pending state.
	if (CellState.ChangeVersion == ChangeId)
	{
		CellState.PendingCancelToken = FRealtimeMeshCancellationToken();
		CellState.PendingFuture.Reset();

		// If we have no mesh, we can recycle the mesh component, but only if we're the latest operation
		if (!bHasMesh && IsValid(CellState.MeshComponent) && CellState.MeshComponent->IsValidLowLevel())
		{
			if (CellState.bShouldBeVisible)
			{
				DeactivateCell(Cell, CellState.MeshComponent);
			}
			UnloadCell(Cell, CellState.MeshComponent->GetRealtimeMesh());
			RecycleMesh(CellState.MeshComponent);
		}
	}
}

void URealtimeMeshSpatialComponent::RecycleMesh(URealtimeMeshComponent* Component)
{
	Component->SetRealtimeMesh(nullptr);
	RecycledMeshes.Push(Component);	
}

void URealtimeMeshSpatialComponent::OnCellChanged(const FRealtimeMeshSpatialComponentLocation& Cell)
{
	const auto ChunkProvider = ChunkProviderWeak.Pin();
	if (!ChunkProvider)
	{
		return;
	}
	
	check(IsInGameThread());

	// Make sure this wasn't unloaded before being fully loaded
	if (!ActiveCells.Contains(Cell))
	{
		return;
	}
		
	auto& CellState = ActiveCells.FindChecked(Cell);
	
	// If we're still waiting on the load, mark the wanted update and bail
	if (!CellState.bIsInitialized || CellState.bIsLoading)
	{
		CellState.bWantsUpdate2 = true;
		return;
	}
	
	check(!CellState.bIsLoading);

	// Cancel any pending operation
	if (CellState.PendingFuture.IsValid())
	{
		CellState.PendingCancelToken.Cancel();
	}

	const int32 ChangeId = ++CellState.ChangeVersion;
	CellState.PendingCancelToken = FRealtimeMeshCancellationToken();
	CellState.bWantsUpdate2 = false;

	if (IsValid(CellState.MeshComponent) && CellState.MeshComponent->IsValidLowLevel())
	{
		CellState.PendingFuture = ContinueOnGameThread(UpdateCell(Cell, CellState.MeshComponent->GetRealtimeMesh(), CellState.PendingCancelToken), [this, Cell, ChangeId, SerialKey = CellState.SerialKey](TFuture<bool>&& UpdateState) mutable
		{
			FinalizeCellUpdate(Cell, ChangeId, UpdateState.Get(), SerialKey);
		});	
	}
	else
	{
		CellState.bIsLoading = true;
		CellState.PendingFuture = ContinueOnGameThread(LoadCell(Cell, CellState.PendingCancelToken), [this, Cell, ChangeId, SerialKey = CellState.SerialKey](TFuture<URealtimeMesh*>&& MeshFuture)
		{
			FinalizeCellLoad(Cell, MeshFuture.Get(), true, ChangeId, SerialKey);
		});
	}	
}

FString URealtimeMeshSpatialComponent::GetCellName(const FRealtimeMeshSpatialComponentLocation& Cell)
{
	static int32 Counter = 0;
	return FString::Printf(TEXT("RealtimeMeshHeightMapCell_%lld_%lld_LOD_%d__Counter:%d"), Cell.Location.X, Cell.Location.Y, Cell.LOD, Counter++);
}

EObjectFlags URealtimeMeshSpatialComponent::GetCellComponentFlags()
{
	if (GetWorld()->IsEditorWorld())
	{
		return RF_Transient;
	}
	else
	{
		return RF_NoFlags;
	}
}


