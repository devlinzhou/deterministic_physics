// Fill out your copyright notice in the Description page of Project Settings.


#include "GPhysicsCollistionActor.h"

#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"
#include "DrawDebugHelpers.h"
#include "glacier_collision_gjk.h"
#include "glacier_debug_draw.h"
#include "GUnrealUtility.h"

class GTempDraw  : public  IGLacierDraw
{
public:
    GTempDraw( UWorld* pWorld)
    {
        m_World = pWorld;
    }


    virtual void DrawLine(const GVector3& V0, const GVector3& V1, uint32_t uColor)
    {
        FVector TV0 = GUtility::Pos_G_to_U( V0 );
        FVector TV1 = GUtility::Pos_G_to_U( V1 );

        UKismetSystemLibrary::DrawDebugLine(m_World, TV0, TV1, FColor::Yellow);
    }


    UWorld* m_World = nullptr;
};

// Sets default values
AGPhysicsCollistionActor::AGPhysicsCollistionActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;
    PrimaryActorTick.bTickEvenWhenPaused = true;
}

// Called when the game starts or when spawned
void AGPhysicsCollistionActor::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AGPhysicsCollistionActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
    UKismetSystemLibrary::DrawDebugSphere(GetWorld(), GetActorLocation(), TestSphereRadius, 18, FColor::Yellow);
    UKismetSystemLibrary::DrawDebugBox(GetWorld(), BoxCenter, BoxHalfSize, FColor::Yellow, BoxRot);
    UKismetSystemLibrary::DrawDebugCapsule(GetWorld(), CapsuleCenter, CapsuleHalfHeight, CapsuleRadius, FQuat::FindBetween(FVector(0, 0, 1), CapsuleDir).Rotator(), FColor::Yellow);
    UKismetSystemLibrary::DrawDebugSphere(GetWorld(), SphereCenter, SphereRadius, 28, FColor::Yellow);

    FColor TriangleColor = FColor::Yellow;
    UKismetSystemLibrary::DrawDebugLine(GetWorld(), Triangle_Pos + Triangle_p0, Triangle_Pos + Triangle_p1, TriangleColor);
    UKismetSystemLibrary::DrawDebugLine(GetWorld(), Triangle_Pos + Triangle_p1, Triangle_Pos + Triangle_p2, TriangleColor);
    UKismetSystemLibrary::DrawDebugLine(GetWorld(), Triangle_Pos + Triangle_p0, Triangle_Pos + Triangle_p2, TriangleColor);

    GShapeSphere ShapeSphere ( GUtility::Pos_U_to_G(TestSphereRadius));
    GShapeBox ShapeBox( GUtility::Pos_U_to_G(BoxHalfSize));


    FQuat UBoxRot = BoxRot.Quaternion();

    GTransform_QT TransShapeA(GQuaternion::Identity(), GUtility::Pos_U_to_G(SphereCenter) );
    GTransform_QT TransShapeB(GUtility::U_to_G(UBoxRot), GUtility::Pos_U_to_G(BoxCenter));

    GTempDraw Tdraw( GetWorld());

    Tdraw.DrawBox( TransShapeB, GVector3::Zero(), GUtility::Pos_U_to_G(BoxHalfSize - FVector(11,11,11)), 0xFFFFFFFF);

    //GCollision_GJK::GJKTest(ShapeSphere, TransShapeA, ShapeBox, TransShapeB, &Tdraw )

    if (GCollision_GJK::GJKTest(ShapeSphere, TransShapeA, ShapeBox, TransShapeB, &Tdraw ))
    {
      //  UKismetSystemLibrary::DrawDebugSphere(GetWorld(), VT, TestSphereRadius + 1.f, 28, FColor::Red);
     //   UKismetSystemLibrary::DrawDebugLine(GetWorld(), VT, VT + OutNormal * 100.f, FColor::Black);
    }


   /* FVector VLocation = GetActorLocation();
    FVector VT;
    FVector OutNormal = FVector(0, 0, 0);


    for (int i = 0; i < m_Convexs.Num() - 1; i++)
    {
        m_Convexs[i].Draw(GetWorld(), FTransform::Identity);

        if (m_Convexs[i].CollisionTest_Sphere(VLocation, TestSphereRadius, VT, &OutNormal))
        {
            UKismetSystemLibrary::DrawDebugSphere(GetWorld(), VT, TestSphereRadius + 1.f, 28, FColor::Red);
            UKismetSystemLibrary::DrawDebugLine(GetWorld(), VT, VT + OutNormal * 100.f, FColor::Black);
        }
    }

    if (m_Convexs.Num() != 0)
    {
        m_SweepConvex.SweepFrom(Convex_Dir, m_Convexs[0]);
        m_SweepConvex.Draw(GetWorld(), FTransform(FVector(0, 0, 1000)));
    }


    if (MFInterset::Sphere_Box(VLocation, TestSphereRadius, BoxCenter, BoxExtern, VT, &OutNormal))
    {
        UKismetSystemLibrary::DrawDebugSphere(GetWorld(), VT, TestSphereRadius + 1.f, 28, FColor::Red);
        UKismetSystemLibrary::DrawDebugLine(GetWorld(), VT, VT + OutNormal * 100.f, FColor::Black);
    }

    //     if (MFInterset::Sphere_Capsule(VLocation, TestSphereRadius, CapsuleP1, CapsuleP2, CapsuleRadius1, VT, &OutNormal))
    //     {
    //         UKismetSystemLibrary::DrawDebugSphere(GetWorld(), VT, TestSphereRadius - 1.f, 18, FColor::Blue);
    //         UKismetSystemLibrary::DrawDebugLine(GetWorld(), VT, VT + OutNormal * 100.f, FColor::Black);
    //     }

    if (MFInterset::Sphere_Capsule(VLocation, TestSphereRadius, CapsuleP1, CapsuleP2, CapsuleRadius1, CapsuleRadius2, VT, &OutNormal))
    {
        UKismetSystemLibrary::DrawDebugSphere(GetWorld(), VT, TestSphereRadius - 1.f, 18, FColor::Blue);
        UKismetSystemLibrary::DrawDebugLine(GetWorld(), VT, VT + OutNormal * 100.f, FColor::Black);
    }


    if (MFInterset::Sphere_Sphere(VLocation, TestSphereRadius, SphereCenterInner, SphereRadiusInner, VT, &OutNormal, true))
    {
        UKismetSystemLibrary::DrawDebugSphere(GetWorld(), VT, TestSphereRadius + 1.f, 28, FColor::Red);
        UKismetSystemLibrary::DrawDebugLine(GetWorld(), VT, VT + OutNormal * 100.f, FColor::Black);
    }

    if (MFInterset::Sphere_Sphere(VLocation, TestSphereRadius, SphereCenterOuter, SphereRadiusOuter, VT, &OutNormal, false))
    {
        UKismetSystemLibrary::DrawDebugSphere(GetWorld(), VT, TestSphereRadius + 1.f, 28, FColor::Red);
        UKismetSystemLibrary::DrawDebugLine(GetWorld(), VT, VT + OutNormal * 100.f, FColor::Black);
    }

    FColor TriangleColor = FColor::Yellow;

    if (MFInterset::Capsule_Triangle(CapsuleP1, CapsuleP2, CapsuleRadius1, VLocation + Triangle_p0, VLocation + Triangle_p1, VLocation + Triangle_p2, VT))
    {
        TriangleColor = FColor::Red;
    }*/

}

