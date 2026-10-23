#include "VoxelOceanFoamTexture.h"
#include "VoxelNoiseTiles.h"
#include "VoxelOceanFoamTable.h"
#include "VoxelLandBake.h"
#include "VoxelPlugin.h"

#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "RenderingThread.h"
#include "RHICommandList.h"
#include "RHIStaticStates.h"
#include "Serialization/Archive.h"
#include "Templates/UniquePtr.h"
#include "Tasks/Task.h"

namespace VoxelOceanFoamTextureDetail
{
    /** Reads and checks the set at Path, builds its table and has the render thread create the
     *  textures into Out (any thread). "" or what's wrong. */
    FString ReadAndUpload(const FString& Path, const TSharedPtr<FVoxelOceanFoamTexture, ESPMode::ThreadSafe>& Out)
    {
        const TUniquePtr<FArchive> File(IFileManager::Get().CreateFileReader(*Path));
        if (!File.IsValid()) return FString::Printf(TEXT("%s isn't there"), *Path);
        const int64 FileSize = File->TotalSize();
        TArray<uint8> Header;
        Header.SetNumZeroed((int32)FMath::Clamp<int64>(FileSize, 0, 4096));
        File->Serialize(Header.GetData(), Header.Num());
        VoxelNoiseTiles::FSet Set;
        uint64_t DataOffset = 0;
        std::string Why = VoxelNoiseTiles::ReadVxntHeader(Header.GetData(), (size_t)Header.Num(), (uint64_t)FMath::Max<int64>(FileSize, 0), Set, DataOffset);
        if (Why.empty()) Why = VoxelNoiseTiles::CheckFoam(Set);
        if (!Why.empty()) return FString::Printf(TEXT("%s: %s"), *Path, UTF8_TO_TCHAR(Why.c_str()));
        Set.Texels.resize((size_t)VoxelNoiseTiles::ChainTexels(Set.N) * 4);
        File->Seek((int64)DataOffset);
        File->Serialize(Set.Texels.data(), (int64)(Set.Texels.size() * 2));
        if (File->IsError()) return FString::Printf(TEXT("%s: the texels couldn't be read"), *Path);

        TSharedPtr<std::vector<float>, ESPMode::ThreadSafe> Table = MakeShared<std::vector<float>, ESPMode::ThreadSafe>();
        VoxelOceanFoamTable::Build(Set, *Table);
        const uint32 N = Set.N;
        const uint32 Mips = Set.Mips;
        Out->N = N;
        Out->Mips = Mips;
        TSharedPtr<std::vector<uint16_t>, ESPMode::ThreadSafe> Texels = MakeShared<std::vector<uint16_t>, ESPMode::ThreadSafe>(MoveTemp(Set.Texels));
        ENQUEUE_RENDER_COMMAND(VoxelOceanFoamUpload)([Out, Texels, Table, N, Mips](FRHICommandListImmediate& RHICmdList)
            {
                const FRHITextureCreateDesc SetDesc = FRHITextureCreateDesc::Create2D(TEXT("VoxelOceanFoamSet"), (int32)N, (int32)N, PF_R16G16B16A16_UNORM)
                    .SetNumMips((uint8)Mips)
                    .SetFlags(ETextureCreateFlags::ShaderResource)
                    .SetInitialState(ERHIAccess::SRVMask);
                Out->Set = RHICmdList.CreateTexture(SetDesc);
                for (uint32 m = 0; m < Mips; m++)
                {
                    const uint32 S = N >> m;
                    const FUpdateTextureRegion2D Region(0, 0, 0, 0, S, S);
                    RHICmdList.UpdateTexture2D(Out->Set, m, Region, S * 8u, (const uint8*)(Texels->data() + VoxelNoiseTiles::MipOffset(N, m) * 4));
                }
                const FRHITextureCreateDesc TableDesc = FRHITextureCreateDesc::Create2D(TEXT("VoxelOceanFoamTable"), VoxelOceanFoamTable::Columns,
                    VoxelOceanFoamTable::Rows(Mips), PF_A32B32G32R32F)
                    .SetFlags(ETextureCreateFlags::ShaderResource)
                    .SetInitialState(ERHIAccess::SRVMask);
                Out->Table = RHICmdList.CreateTexture(TableDesc);
                const FUpdateTextureRegion2D TableRegion(0, 0, 0, 0, (uint32)VoxelOceanFoamTable::Columns, (uint32)VoxelOceanFoamTable::Rows(Mips));
                RHICmdList.UpdateTexture2D(Out->Table, 0, TableRegion, (uint32)VoxelOceanFoamTable::Columns * 16u, (const uint8*)Table->data());
                Out->bReady = true;
            });
        return FString();
    }
}

TSharedPtr<FVoxelOceanFoamTexture, ESPMode::ThreadSafe> VoxelOceanFoamTexture::Load(const FString& Name, FString& OutError)
{
    const FString Folder = VoxelLand::NoiseSetDirectory();
    if (Folder.IsEmpty())
    {
        OutError = TEXT("the plugin's NoiseSets folder wasn't found");
        return nullptr;
    }
    const FString Path = FPaths::Combine(Folder, Name + TEXT(".vxnt"));
    if (!IFileManager::Get().FileExists(*Path))
    {
        OutError = FString::Printf(TEXT("%s isn't there"), *Path);
        return nullptr;
    }
    // The read and the thresholds' build (about a second at 1024) off the game thread; the
    // procedural lace stands in until the textures exist (bReady).
    TSharedPtr<FVoxelOceanFoamTexture, ESPMode::ThreadSafe> Out = MakeShared<FVoxelOceanFoamTexture, ESPMode::ThreadSafe>();
    Out->Name = Name;
    UE::Tasks::Launch(UE_SOURCE_LOCATION, [Out, Path, Name]()
        {
            const double Start = FPlatformTime::Seconds();
            const FString Error = VoxelOceanFoamTextureDetail::ReadAndUpload(Path, Out);
            // (Braced: UE_LOG is a block, so an if / else around it needs them.)
            if (Error.IsEmpty())
            {
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Read the foam set %s (%u^2, %u mips) in %.2f s."), *Name, Out->N, Out->Mips, FPlatformTime::Seconds() - Start);
            }
            else
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] The foam set couldn't be read (%s); the procedural lace stands in."), *Error);
            }
        });
    return Out;
}
