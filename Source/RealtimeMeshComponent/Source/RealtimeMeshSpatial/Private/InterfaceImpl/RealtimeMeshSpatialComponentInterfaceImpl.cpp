// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.

#include "RealtimeMeshSpatialFactoryComponent.h"
#include "Core/RealtimeMeshModularFeatures.h"
#include "RealtimeMeshExt/Private/InterfaceImpl/RealtimeMeshFactoryInterfaceImpl.h"
#include "Spatial/RealtimeMeshSpatialComponentInterface.h"


namespace RealtimeMesh
{
	class FRealtimeMeshStreamingPolicyImpl_v0;
	
	struct FRealtimeMeshSpatialComponentInterfaceImpl_v0 : public IRealtimeMeshSpatialComponentInterface_v0
	{
	public:
		virtual USceneComponent* CreateComponent(UObject* Owner, const TSharedRef<IRealtimeMeshSpatialStreamingFactoryStructureProvider>& InChunkProvider, const TSharedPtr<IRealtimeMeshFactory_v0>& InFactory) const override
		{
			URealtimeMeshSpatialFactoryComponent* SpatialComp = NewObject<URealtimeMeshSpatialFactoryComponent>(Owner);
			if (InFactory)
			{
				SpatialComp->Initialize(InChunkProvider, StaticCastSharedPtr<FRealtimeMeshFactoryImpl_v0>(InFactory)->GetFactory());
			}

			return SpatialComp;
		}
	};

	// Register the interface
	TRealtimeMeshModularFeatureRegistration<FRealtimeMeshSpatialComponentInterfaceImpl_v0> GRealtimeMeshSpatialComponentInterfaceImpl_v0;
}
