// Author: Antonio Lattanzio - emptyvessel

#include "Box3DStaticGeometry.h"
#include "Box3DCollisionData.h"
#include "Box3DConversion.h"
#include "Box3DLog.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/ShapeComponent.h"
#include "LandscapeHeightfieldCollisionComponent.h"
#include "Chaos/HeightField.h"
#include "Chaos/TriangleMeshImplicitObject.h"
#include "Engine/World.h"
#include "Engine/Level.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "Interfaces/Interface_CollisionDataProvider.h"
#include "Interface_CollisionDataProviderCore.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"

namespace Box3D::StaticGeometry
{
	namespace
	{
		constexpr int32 MaxHullVertices = 64;

		// Local vertex (cm) -> box3d point (m): bake scale, negate Y (see Box3DConversion.h).
		FORCEINLINE FVector3f LocalToBaked(const FVector& V, const FVector& Scale)
		{
			return FVector3f(
				static_cast<float>(V.X * Scale.X * Box3D::UnrealToMeters),
				static_cast<float>(-V.Y * Scale.Y * Box3D::UnrealToMeters),
				static_cast<float>(V.Z * Scale.Z * Box3D::UnrealToMeters));
		}

		FORCEINLINE b3Vec3 ToB3(const FVector3f& V) { return b3Vec3{ V.X, V.Y, V.Z }; }

		FORCEINLINE bool ShouldReverseWinding(const FVector& Scale, bool bInvert)
		{
			const bool bNegativeScale = (Scale.X * Scale.Y * Scale.Z) < 0.0;
			return bNegativeScale ^ bInvert;
		}

		// Move an already-baked shape by an Unreal-space offset. The shape's points are in box3d
		// space (Y negated, metres), so the offset is converted the same way rather than applied raw.
		void PlaceShape(FBox3DBakedShape& Shape, const FTransform& ComponentToActor)
		{
			auto Move = [&ComponentToActor](const FVector3f& P)
			{
				const FVector Unreal(P.X * Box3D::MetersToUnreal,
					-P.Y * Box3D::MetersToUnreal, P.Z * Box3D::MetersToUnreal);
				const FVector Moved = ComponentToActor.TransformPosition(Unreal);
				return FVector3f(
					static_cast<float>(Moved.X * Box3D::UnrealToMeters),
					static_cast<float>(-Moved.Y * Box3D::UnrealToMeters),
					static_cast<float>(Moved.Z * Box3D::UnrealToMeters));
			};

			for (FVector3f& Point : Shape.Points)
			{
				Point = Move(Point);
			}
			if (Shape.Kind == EBox3DBakedShapeKind::Sphere
				|| Shape.Kind == EBox3DBakedShapeKind::Capsule)
			{
				Shape.CenterA = Move(Shape.CenterA);
				Shape.CenterB = Move(Shape.CenterB);
			}
		}

		IInterface_CollisionDataProvider* FindTriMeshProvider(UPrimitiveComponent* Prim)
		{
			// Landscape collision components implement the provider directly.
			if (IInterface_CollisionDataProvider* Direct = Cast<IInterface_CollisionDataProvider>(Prim))
			{
				return Direct;
			}
			if (const UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Prim))
			{
				if (UStaticMesh* Mesh = SMC->GetStaticMesh())
				{
					return Cast<IInterface_CollisionDataProvider>(Mesh);
				}
			}
			return nullptr;
		}

		// --- Stage 1 helpers: UE collision -> FBox3DBakedShape (box3d local space) ---------

		bool AppendHull(TArray<FBox3DBakedShape>& OutShapes, const TArray<FVector>& Points, const FVector& Scale)
		{
			if (Points.Num() < 4)
			{
				return false;
			}

			FBox3DBakedShape Shape;
			Shape.Kind = EBox3DBakedShapeKind::Hull;
			Shape.Points.Reserve(Points.Num());
			for (const FVector& P : Points)
			{
				Shape.Points.Add(LocalToBaked(P, Scale));
			}

			// Validate now (in the editor) so a bad cloud fails visibly here, not at load.
			TArray<b3Vec3> Converted;
			Converted.Reserve(Shape.Points.Num());
			for (const FVector3f& P : Shape.Points)
			{
				Converted.Add(ToB3(P));
			}
			b3HullData* Hull = b3CreateHull(Converted.GetData(), Converted.Num(), MaxHullVertices);
			if (Hull == nullptr)
			{
				return false;
			}
			b3DestroyHull(Hull);

			OutShapes.Add(MoveTemp(Shape));
			return true;
		}

		// Simple collision: AggGeom convex/box/sphere/capsule -> baked shapes. Returns count.
		int32 ExtractSimpleCollision(UPrimitiveComponent* Prim, const FVector& Scale, TArray<FBox3DBakedShape>& OutShapes)
		{
			UBodySetup* Setup = Prim->GetBodySetup();
			if (Setup == nullptr)
			{
				return 0;
			}

			const FKAggregateGeom& Agg = Setup->AggGeom;
			int32 Created = 0;

			// Convex: element transform places local verts into body space.
			for (const FKConvexElem& Convex : Agg.ConvexElems)
			{
				const FTransform ElemTM = Convex.GetTransform();
				TArray<FVector> Points;
				Points.Reserve(Convex.VertexData.Num());
				for (const FVector& V : Convex.VertexData)
				{
					Points.Add(ElemTM.TransformPosition(V));
				}
				if (!AppendHull(OutShapes, Points, Scale)) { OutShapes.Reset(); return 0; }
                ++Created;
			}

			// Boxes: 8 corners through the hull path.
			for (const FKBoxElem& Box : Agg.BoxElems)
			{
				const FTransform ElemTM(Box.Rotation, Box.Center);
				const FVector He(Box.X * 0.5f, Box.Y * 0.5f, Box.Z * 0.5f);
				TArray<FVector> Corners;
				Corners.Reserve(8);
				for (int32 Sx = -1; Sx <= 1; Sx += 2)
				for (int32 Sy = -1; Sy <= 1; Sy += 2)
				for (int32 Sz = -1; Sz <= 1; Sz += 2)
				{
					Corners.Add(ElemTM.TransformPosition(FVector(Sx * He.X, Sy * He.Y, Sz * He.Z)));
				}
				if (!AppendHull(OutShapes, Corners, Scale)) { OutShapes.Reset(); return 0; }
                ++Created;
			}

			// Non-uniform scale can't stay round, so radii use the min axis scale.
			const float RadialScale = static_cast<float>(Scale.GetAbsMin());
			const float M = static_cast<float>(Box3D::UnrealToMeters);
			for (const FKSphereElem& Sph : Agg.SphereElems)
			{
				FBox3DBakedShape Shape;
				Shape.Kind = EBox3DBakedShapeKind::Sphere;
				Shape.CenterA = LocalToBaked(Sph.Center, Scale);
				Shape.Radius = Sph.Radius * RadialScale * M;
				OutShapes.Add(MoveTemp(Shape));
				++Created;
			}

			// Sphyls: local-Z axis, Length spans the two hemisphere centers.
			for (const FKSphylElem& Capsule : Agg.SphylElems)
			{
				const FKSphylElem ScaledCapsule = Capsule.GetFinalScaled(Scale, FTransform::Identity);
                const FTransform ElemTM(ScaledCapsule.Rotation, ScaledCapsule.Center);
				const float HalfLen = ScaledCapsule.Length * 0.5f;
				FBox3DBakedShape Shape;
				Shape.Kind = EBox3DBakedShapeKind::Capsule;
				Shape.CenterA = LocalToBaked(ElemTM.TransformPosition(FVector(0, 0, +HalfLen)), FVector::OneVector);
				Shape.CenterB = LocalToBaked(ElemTM.TransformPosition(FVector(0, 0, -HalfLen)), FVector::OneVector);
				Shape.Radius = ScaledCapsule.Radius * M;
				OutShapes.Add(MoveTemp(Shape));
				++Created;
			}

			return Created;
		}

		bool ExtractComplexTriMesh(
			IInterface_CollisionDataProvider* Provider, const FVector& Scale, bool bInvertWinding,
			const FTransform& ComponentToActor, TArray<FBox3DBakedShape>& OutShapes)
		{
			if (Provider == nullptr || !Provider->ContainsPhysicsTriMeshData(true))
			{
				return false;
			}

			FTriMeshCollisionData TriData;
			if (!Provider->GetPhysicsTriMeshData(&TriData, /*InUseAllTriData=*/true))
			{
				return false;
			}
			if (TriData.Vertices.Num() < 3 || TriData.Indices.Num() < 1)
			{
				return false;
			}

			// The component's own scale is already applied here, so its relative transform is
			// taken rotation-and-translation only or the scale would be counted twice.
			const FTransform Placement(ComponentToActor.GetRotation(),
				ComponentToActor.GetTranslation(), FVector::OneVector);

			FBox3DBakedShape Shape;
			Shape.Kind = EBox3DBakedShapeKind::Mesh;
			Shape.Points.Reserve(TriData.Vertices.Num());
			for (const FVector3f& V : TriData.Vertices)
			{
				const FVector Scaled(V.X * Scale.X, V.Y * Scale.Y, V.Z * Scale.Z);
				Shape.Points.Add(LocalToBaked(Placement.TransformPosition(Scaled), FVector::OneVector));
			}

			const bool bReverse = ShouldReverseWinding(Scale, bInvertWinding);
			Shape.Indices.Reserve(TriData.Indices.Num() * 3);
			for (const FTriIndices& Tri : TriData.Indices)
			{
				Shape.Indices.Add(Tri.v0);
				if (bReverse)
				{
					Shape.Indices.Add(Tri.v2);
					Shape.Indices.Add(Tri.v1);
				}
				else
				{
					Shape.Indices.Add(Tri.v1);
					Shape.Indices.Add(Tri.v2);
				}
			}

			OutShapes.Add(MoveTemp(Shape));
			return true;
		}

		// --- Stage 2 helpers: FBox3DBakedShape -> box3d shape on a body -------------------

		void InstantiateHull(b3BodyId Body, const b3ShapeDef& Def, const FBox3DBakedShape& Shape)
		{
			if (Shape.Points.Num() < 4)
			{
				return;
			}
			TArray<b3Vec3> Points;
			Points.Reserve(Shape.Points.Num());
			for (const FVector3f& P : Shape.Points)
			{
				Points.Add(ToB3(P));
			}
			b3HullData* Hull = b3CreateHull(Points.GetData(), Points.Num(), MaxHullVertices);
			if (Hull == nullptr)
			{
				return;
			}
			b3CreateHullShape(Body, &Def, Hull); // box3d clones the hull; free ours after
			b3DestroyHull(Hull);
		}

		// Returns the b3MeshData the caller must free after the body (nullptr on failure).
		b3MeshData* InstantiateMesh(b3BodyId Body, const b3ShapeDef& Def, const FBox3DBakedShape& Shape)
		{
			if (Shape.Points.Num() < 3 || Shape.Indices.Num() < 3)
			{
				return nullptr;
			}

			TArray<b3Vec3> Vertices;
			Vertices.Reserve(Shape.Points.Num());
			for (const FVector3f& P : Shape.Points)
			{
				Vertices.Add(ToB3(P));
			}

			b3MeshDef MeshDef{};
			MeshDef.vertices = Vertices.GetData();
			MeshDef.indices = const_cast<int32*>(Shape.Indices.GetData());
			MeshDef.materialIndices = nullptr; // base material for all triangles
			MeshDef.weldTolerance = 0.0f;
			MeshDef.vertexCount = Vertices.Num();
			MeshDef.triangleCount = Shape.Indices.Num() / 3;
			MeshDef.weldVertices = false;
			MeshDef.useMedianSplit = false;
			MeshDef.identifyEdges = true;   // adjacency avoids ghost collisions

			b3MeshData* Mesh = b3CreateMesh(&MeshDef, nullptr, 0);
			if (Mesh == nullptr)
			{
				return nullptr;
			}

			// Scale already baked into verts, so unit shape scale.
			b3CreateMeshShape(Body, &Def, Mesh, b3Vec3{ 1.0f, 1.0f, 1.0f });
			return Mesh;
		}
	} // namespace

	bool ExtractStaticCollision(AActor* Owner, ESource Source, bool bInvertWinding, FBox3DBakedBody& Out, bool bStaticOnly)
	{
		if (Owner == nullptr)
		{
			return false;
		}

		Out.WorldTransform = Owner->GetActorTransform();
		Out.ActorKey = Owner->GetPathName();

		// Read authored static collision on all children, foliage instances and landscape sections.
		TArray<UPrimitiveComponent*> Primitives;
		Owner->GetComponents<UPrimitiveComponent>(Primitives);
        Primitives.RemoveAll([bStaticOnly](const UPrimitiveComponent* Prim)
        {
            return !Prim || !Prim->IsCollisionEnabled() || Prim->GetCollisionResponseToChannel(ECC_Pawn) != ECR_Block ||
                (bStaticOnly && (Prim->Mobility == EComponentMobility::Movable || Prim->IsSimulatingPhysics() ||
                (!Prim->IsA<UStaticMeshComponent>() && !Prim->IsA<ULandscapeHeightfieldCollisionComponent>() && !Prim->IsA<UShapeComponent>())));
        });
        Primitives.Sort([](const UPrimitiveComponent& A, const UPrimitiveComponent& B)
        {
            return A.GetPathName() < B.GetPathName();
        });
		// Quiet: an actor with no primitives is usually a deliberate one carrying an explicit
		// primitive Shape (a bare static pad, for instance), and the caller decides whether that
		// is a problem. It only matters for Shape=Auto, which warns for itself.
		if (Primitives.IsEmpty())
		{
			return false;
		}

		int32 CollisionEnabledCount = 0;
		for (const UPrimitiveComponent* Prim : Primitives)
		{
			CollisionEnabledCount += (Prim != nullptr && Prim->IsCollisionEnabled()) ? 1 : 0;
		}
		if (CollisionEnabledCount == 0)
		{
			UE_LOG(LogBox3D, Warning,
				TEXT("%s: has %d primitive component(s) but collision is disabled on all of them. ")
				TEXT("Set the component's Collision Enabled to at least 'Query and Physics'."),
				*GetNameSafe(Owner), Primitives.Num());
			return false;
		}

		// A child's shape is baked in the actor's space, so its own offset from the actor has to
		// come along or every landscape section stacks up at the origin.
		const FTransform ActorToWorld = Owner->GetActorTransform();

        // Match Unreal's simple-query collision choice per component. A mixed
        // actor must retain both its authored simple and complex components.
        for(UPrimitiveComponent* Prim:Primitives)
        {
            if(auto* Landscape=Cast<ULandscapeHeightfieldCollisionComponent>(Prim))
            {
                if(!Landscape->HeightfieldRef && !Landscape->LocalHeightfieldGeometryRef) Landscape->CreateCollisionObject();
                auto Field=Landscape->HeightfieldRef ? Landscape->HeightfieldRef->HeightfieldGeometry : Landscape->LocalHeightfieldGeometryRef;
                if(!Field) { UE_LOG(LogBox3D,Error,TEXT("Landscape %s has no loaded cooked collision."),*Prim->GetPathName()); return false; }
                FBox3DBakedShape Shape; Shape.Kind=EBox3DBakedShapeKind::Mesh;
                const int32 Rows=Field->GetNumRows(), Cols=Field->GetNumCols();
                const FTransform Placement=FTransform(Prim->GetComponentQuat(),Prim->GetComponentLocation()).GetRelativeTransform(FTransform(ActorToWorld.GetRotation(),ActorToWorld.GetTranslation()));
                for(int32 I=0;I<Rows*Cols;++I) Shape.Points.Add(LocalToBaked(FVector(Field->GetPointScaled(I)),FVector::OneVector));
                for(int32 R=0;R<Rows-1;++R) for(int32 C=0;C<Cols-1;++C)
                {
                    if(Field->IsHole(C,R)) continue;
                    for(int32 T=0;T<2;++T)
                    {
                        Chaos::FTriangle Triangle; int32 A,B,CIndex;
                        Field->GetTransformedTriangle(2*(R*(Cols-1)+C)+T,Chaos::FRigidTransform3::Identity,Triangle,A,B,CIndex);
                        Shape.Indices.Append({A,CIndex,B}); // Unreal -> Box3D reflection reverses winding.
                    }
                }
                PlaceShape(Shape,Placement); Out.Shapes.Add(MoveTemp(Shape));
                continue;
            }
            UBodySetup* Setup=Prim->GetBodySetup();
            const bool Complex=Source==ESource::ComplexCollision ||
                (Source==ESource::Auto && Setup && (Setup->GetCollisionTraceFlag()==CTF_UseComplexAsSimple ||
                (Setup->GetCollisionTraceFlag()==CTF_UseSimpleAndComplex && Setup->AggGeom.GetElementCount()==0)));
            TArray<FTransform> Placements;
            if(auto* Instances=Cast<UInstancedStaticMeshComponent>(Prim))
            {
                for(int32 I=0;I<Instances->GetInstanceCount();++I)
                { FTransform Instance; if(Instances->GetInstanceTransform(I,Instance,true)) Placements.Add(Instance); }
            }
            else Placements.Add(Prim->GetComponentTransform());
            for(const FTransform& Instance:Placements)
            {
                const int32 Before=Out.Shapes.Num();
                const FTransform Relative=Instance.GetRelativeTransform(FTransform(ActorToWorld.GetRotation(),ActorToWorld.GetTranslation()));
                const FTransform Placement(Relative.GetRotation(),Relative.GetTranslation(),FVector::OneVector);
                if(Complex && Setup)
                {
                    // Read cooked physics geometry: works without Allow CPU Access or editor mesh data.
                    Setup->CreatePhysicsMeshes();
                    for(const auto& Mesh:Setup->TriMeshGeometries)
                    {
                        if(!Mesh) continue;
                        FBox3DBakedShape Shape; Shape.Kind=EBox3DBakedShapeKind::Mesh;
                        const auto& Particles=Mesh->Particles();
                        for(int32 I=0;I<int32(Particles.Size());++I) Shape.Points.Add(LocalToBaked(FVector(Particles.GetX(I)),Instance.GetScale3D()));
                        for(int32 I=0;I<Mesh->Elements().GetNumTriangles();++I)
                        {
                            Chaos::FTriangle Triangle; int32 A,B,C;
                            Mesh->GetTransformedTriangle(I,Chaos::FRigidTransform3::Identity,Triangle,A,B,C);
                            if(!ShouldReverseWinding(Instance.GetScale3D(),bInvertWinding)) Shape.Indices.Append({A,C,B});
                            else Shape.Indices.Append({A,B,C});
                        }
                        PlaceShape(Shape,Placement); Out.Shapes.Add(MoveTemp(Shape));
                    }
                }
                else
                {
                    ExtractSimpleCollision(Prim,Instance.GetScale3D(),Out.Shapes);
                    for(int32 Index=Before;Index<Out.Shapes.Num();++Index) PlaceShape(Out.Shapes[Index],Placement);
                }
                if(Out.Shapes.Num()==Before)
                {
                    UE_LOG(LogBox3D,Error,TEXT("%s: Pawn-blocking component has no usable authored collision."),*Prim->GetPathName());
                    Out.Shapes.Reset(); return false;
                }
            }
        }
        if(!Out.Shapes.IsEmpty())return true;
		UE_LOG(LogBox3D, Warning,
			TEXT("%s: no cooked collision found for static body (no tri-mesh, no simple primitives). ")
			TEXT("A landscape always lands here: its collision is a Chaos heightfield, which this ")
			TEXT("does not read yet. For a mesh, add simple collision or enable complex collision."),
			*GetNameSafe(Owner));
		return false;
	}

    bool ExtractSceneCollision(UWorld* World,TArray<FBox3DBakedBody>& Out)
    {
        Out.Reset(); if(!World) return false;
        TArray<AActor*> Actors;
        for(ULevel* Level:World->GetLevels()) if(Level) for(AActor* Actor:Level->Actors)
        {
            if(!IsValid(Actor) || Actor->IsActorBeingDestroyed()) continue;
            bool bExplicitBody=false;
            for(UClass* Class=Actor->GetClass();Class;Class=Class->GetSuperClass())
                if(Class->GetFName()==FName(TEXT("RollbackPhysicsProp"))) bExplicitBody=true;
            TArray<UActorComponent*> Components; Actor->GetComponents(Components);
            for(auto* Component:Components) for(UClass* Class=Component->GetClass();Class;Class=Class->GetSuperClass())
                if(Class->GetFName()==FName(TEXT("Box3DBodyComponent"))) bExplicitBody=true;
            if(!bExplicitBody) Actors.Add(Actor);
        }
        Actors.Sort([](const AActor& A,const AActor& B){return UWorld::RemovePIEPrefix(A.GetPathName())<UWorld::RemovePIEPrefix(B.GetPathName());});
        for(auto* Actor:Actors)
        {
            TArray<UPrimitiveComponent*> Primitives; Actor->GetComponents(Primitives);
            bool bCandidate=false;
            for(auto* Prim:Primitives)
                if(Prim->Mobility!=EComponentMobility::Movable && !Prim->IsSimulatingPhysics() && Prim->IsCollisionEnabled() &&
                   Prim->GetCollisionResponseToChannel(ECC_Pawn)==ECR_Block &&
                   (Prim->IsA<UStaticMeshComponent>() || Prim->IsA<ULandscapeHeightfieldCollisionComponent>() || Prim->IsA<UShapeComponent>()))
                { if(auto* Instances=Cast<UInstancedStaticMeshComponent>(Prim)) bCandidate|=Instances->GetInstanceCount()>0; else bCandidate=true; }
            if(!bCandidate) continue;
            FBox3DBakedBody Body;
            if(ExtractStaticCollision(Actor,ESource::Auto,false,Body,true))
            { Body.ActorKey=UWorld::RemovePIEPrefix(Actor->GetPathName()); Out.Add(MoveTemp(Body)); }
            else return false;
        }
        return true;
    }

	bool AddBakedShapes(
		b3BodyId Body, const b3ShapeDef& Base, const FBox3DBakedBody& Baked, TArray<b3MeshData*>& OutOwnedMeshes)
	{
		int32 Created = 0;
		for (const FBox3DBakedShape& Shape : Baked.Shapes)
		{
			switch (Shape.Kind)
			{
			case EBox3DBakedShapeKind::Hull:
				InstantiateHull(Body, Base, Shape);
				++Created;
				break;
			case EBox3DBakedShapeKind::Mesh:
                if(Shape.Indices.IsEmpty()) { ++Created; break; }
				if (b3MeshData* Mesh = InstantiateMesh(Body, Base, Shape))
				{
					OutOwnedMeshes.Add(Mesh);
					++Created;
				}
				break;
			case EBox3DBakedShapeKind::Sphere:
			{
				b3Sphere Sphere;
				Sphere.center = ToB3(Shape.CenterA);
				Sphere.radius = Shape.Radius;
				b3CreateSphereShape(Body, &Base, &Sphere);
				++Created;
				break;
			}
			case EBox3DBakedShapeKind::Capsule:
			{
				b3Capsule Capsule;
				Capsule.center1 = ToB3(Shape.CenterA);
				Capsule.center2 = ToB3(Shape.CenterB);
				Capsule.radius = Shape.Radius;
				b3CreateCapsuleShape(Body, &Base, &Capsule);
				++Created;
				break;
			}
			}
		}
		return Created > 0;
	}

	bool AddStaticShapes(
		b3BodyId Body, const b3ShapeDef& Base, AActor* Owner, ESource Source,
		bool bInvertWinding, TArray<b3MeshData*>& OutOwnedMeshes)
	{
		FBox3DBakedBody Baked;
		if (!ExtractStaticCollision(Owner, Source, bInvertWinding, Baked))
		{
			return false;
		}
		return AddBakedShapes(Body, Base, Baked, OutOwnedMeshes);
	}

	FString GetBox3DVersionString()
	{
		const b3Version V = b3GetVersion();
		return FString::Printf(TEXT("%d.%d.%d"), V.major, V.minor, V.revision);
	}
} // namespace Box3D::StaticGeometry
