// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.


#include "RealtimeMeshFactoryInterfaceImpl.h"
#include "Factory/RealtimeMeshFactory.h"
#include "Core/RealtimeMeshModularFeatures.h"

namespace RealtimeMesh
{

	FRealtimeMeshFactoryImpl_v0::FRealtimeMeshFactoryImpl_v0(const TSharedRef<FRealtimeMeshFactory>& InFactory)
		: Factory(InFactory)
	{ }


	void FRealtimeMeshFactoryInterfaceImpl_v0::CleanupDanglingReferences() const
	{
		for (auto It = FactoryMap.CreateIterator(); It; ++It)
		{
			if (!It.Value().IsValid())
			{
				It.RemoveCurrent();
			}
		}
	}
	
	TSharedRef<IRealtimeMeshFactory_v0> FRealtimeMeshFactoryInterfaceImpl_v0::CreateFactory(const TSharedRef<FRealtimeMeshFactoryProvider>& InProvider) const
	{
		TSharedRef<FRealtimeMeshFactory> Factory = MakeShared<FRealtimeMeshFactory>(InProvider);
		TSharedRef<FRealtimeMeshFactoryImpl_v0> FactoryInterface = MakeShared<FRealtimeMeshFactoryImpl_v0>(Factory);
		FactoryMap.Add(Factory, FactoryInterface);	
		return FactoryInterface;
	}

	// Register the interface
	TRealtimeMeshModularFeatureRegistration<FRealtimeMeshFactoryInterfaceImpl_v0> GRealtimeMeshFactoryInterfaceImpl_v0;
}
