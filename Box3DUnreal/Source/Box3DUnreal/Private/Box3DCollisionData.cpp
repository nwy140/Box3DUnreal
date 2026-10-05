// Author: Antonio Lattanzio - emptyvessel

#include "Box3DCollisionData.h"
#include "Serialization/MemoryWriter.h"
#include "Misc/SecureHash.h"
#include "Box3DStaticGeometry.h"
#include "Misc/PackageName.h"

#if WITH_EDITOR
#include "Engine/Level.h"
#include "HAL/FileManager.h"
#endif

FString UBox3DCollisionData::DeriveAssetPackageName(const FString& MapPackageName)
{
	const FString Path = FPackageName::GetLongPackagePath(MapPackageName); // /Game/Maps
	const FString Name = FPackageName::GetShortName(MapPackageName);       // Foo
	return FString::Printf(TEXT("%s/BC_%s"), *Path, *Name);
}

#if WITH_EDITOR

FString UBox3DCollisionData::ComputeSourceFingerprint(const FString& MapPackageName)
{
	FString MapFilename;
	if (!FPackageName::TryConvertLongPackageNameToFilename(
			MapPackageName, MapFilename, FPackageName::GetMapPackageExtension()))
	{
		return FString();
	}

	IFileManager& Files = IFileManager::Get();
	if (!Files.FileExists(*MapFilename))
	{
		return FString();
	}

	FDateTime Newest = Files.GetTimeStamp(*MapFilename);
	int64 TotalSize = Files.FileSize(*MapFilename);
	int32 FileCount = 1;

	const FString ExternalActorsPath = ULevel::GetExternalActorsPath(MapPackageName);
	FString ExternalActorsDir;
	if (!ExternalActorsPath.IsEmpty()
		&& FPackageName::TryConvertLongPackageNameToFilename(ExternalActorsPath, ExternalActorsDir)
		&& Files.DirectoryExists(*ExternalActorsDir))
	{
		Files.IterateDirectoryStatRecursively(*ExternalActorsDir,
			[&Newest, &TotalSize, &FileCount](const TCHAR*, const FFileStatData& Stat) -> bool
			{
				if (!Stat.bIsDirectory)
				{
					Newest = FMath::Max(Newest, Stat.ModificationTime);
					TotalSize += Stat.FileSize;
					++FileCount;
				}
				return true;
			});
	}

	return FString::Printf(TEXT("%lld|%lld|%d"), Newest.GetTicks(), TotalSize, FileCount);
}

bool UBox3DCollisionData::IsStale(FString& OutReason) const
{
	const FString CurrentVersion = Box3D::StaticGeometry::GetBox3DVersionString();
	if (!Box3DVersion.IsEmpty() && Box3DVersion != CurrentVersion)
	{
		OutReason = FString::Printf(TEXT("baked against box3d %s, now running %s"), *Box3DVersion, *CurrentVersion);
		return true;
	}

	if (SourceFingerprint.IsEmpty())
	{
		OutReason = TEXT("no source fingerprint recorded (baked before staleness checks existed)");
		return true;
	}

	const FString Current = ComputeSourceFingerprint(SourceLevel);
	if (Current.IsEmpty())
	{
		OutReason = FString::Printf(TEXT("source level '%s' not found on disk"), *SourceLevel);
		return true;
	}
	if (Current != SourceFingerprint)
	{
		OutReason = FString::Printf(TEXT("source level '%s' changed since the bake"), *SourceLevel);
		return true;
	}

	return false;
}

#endif // WITH_EDITOR

// Field serialization avoids struct padding, pointers and bake timestamps.
FString UBox3DCollisionData::ComputeGeometryDigest() const
{
    TArray<uint8> Bytes;
    FMemoryWriter Writer(Bytes, true);
    uint32 Schema = 1;
    Writer << Schema;
    int32 Count = Bodies.Num();
    Writer << Count;
    for (const auto& Body : Bodies)
    {
        FString Key = Body.ActorKey;
        Writer << Key;
        const FVector P = Body.WorldTransform.GetTranslation();
        const FQuat Q = Body.WorldTransform.GetRotation();
        const FVector S = Body.WorldTransform.GetScale3D();
        for (double Value : {P.X,P.Y,P.Z,Q.X,Q.Y,Q.Z,Q.W,S.X,S.Y,S.Z}) Writer << Value;
        Count = Body.Shapes.Num(); Writer << Count;
        for (const auto& Shape : Body.Shapes)
        {
            uint8 Kind = static_cast<uint8>(Shape.Kind); Writer << Kind;
            Count = Shape.Points.Num(); Writer << Count;
            for (const auto& Point : Shape.Points)
                for (float Value : {Point.X,Point.Y,Point.Z}) Writer << Value;
            Count = Shape.Indices.Num(); Writer << Count;
            for (int32 Index : Shape.Indices) Writer << Index;
            for (float Value : {Shape.CenterA.X,Shape.CenterA.Y,Shape.CenterA.Z,
                Shape.CenterB.X,Shape.CenterB.Y,Shape.CenterB.Z,Shape.Radius}) Writer << Value;
        }
    }
    uint8 Digest[20];
    FSHA1::HashBuffer(Bytes.GetData(), Bytes.Num(), Digest);
    return BytesToHex(Digest, 20);
}
