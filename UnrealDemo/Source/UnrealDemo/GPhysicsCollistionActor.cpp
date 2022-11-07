// Fill out your copyright notice in the Description page of Project Settings.


#include "GPhysicsCollistionActor.h"

#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"
#include "DrawDebugHelpers.h"
#include "glacier_collision_gjk.h"
#include "glacier_collision_box.h"
#include "glacier_collision_sphere.h"
#include "glacier_debug_draw.h"
#include "glacier_convexhull.h"
#include "glacier_contact.h"
#include "glacier_matrix.h"
#include "glacier_physics_utils.h"

#include "GUnrealUtility.h"



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

    pConvexHullA = new GConvexHull();
    pConvexHullB = new GConvexHull();


    std::vector<GVector3> PossA;
    std::vector<GVector3> PossB;


    FVector UBoxHalf = FVector(CovexRandomSize,CovexRandomSize,CovexRandomSize);

    GVector3 VMin = GUtility::Unit_U_to_G( -UBoxHalf);
    GVector3 VMax = GUtility::Unit_U_to_G(UBoxHalf);


    for( int i = 0; i < CovexRandomCount; ++i )
    {
        FVector TPos = UKismetMathLibrary::RandomPointInBoundingBox( FVector(0,0,0), FVector(CovexRandomSize,CovexRandomSize,CovexRandomSize));     
       // PossA.push_back( GUtility::Unit_U_to_G(TPos) );
    }
    
    GConvexHullBuilder::AddBoxPoints(PossA, VMin, VMax);

    pBuilder = new GConvexHullBuilder();
    pBuilder->BuildConvex( PossA, *pConvexHullA );

    pConvexHullA->BuildEdges();


    GConvexHullBuilder::AddBoxPoints( PossB, VMin, VMax );


    pBuilder->BuildConvex( PossB, *pConvexHullB );

     pConvexHullB->BuildEdges();

}

// Called every frame
void AGPhysicsCollistionActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

    GQuaternion TRot( GVector3( GMath::One(), -GMath::Two(), -GMath::Three()).GetNormalize(), GMath::Two() );


    GMatrix3 M3(TRot);

    GQuaternion Rot3 = M3.ToQuat();

    GPhysicsDraw       Tdraw(GetWorld());

    GShapeSphere    ShapeSphere(GUtility::Unit_U_to_G(TestSphereRadius));
    GTransform_QT   TransSphere(GQuaternion::Identity(), GUtility::Unit_U_to_G(GetActorLocation()));

    if( BoxShow )
    {
        GShapeBox ShapeBoxA(GUtility::Unit_U_to_G(BoxHalfSizeA));
        GShapeBox ShapeBoxB(GUtility::Unit_U_to_G(BoxHalfSizeB));

        FVector BoxCenterA = GetActorLocation();
        FRotator BoxRotA = GetActorRotation();

        GTransform_QT TBoxShapeA(GUtility::U_to_G(BoxRotA.Quaternion()), GUtility::Unit_U_to_G(BoxCenterA));
        GTransform_QT TBoxShapeB(GUtility::U_to_G(BoxRotB.Quaternion()), GUtility::Unit_U_to_G(BoxCenterB));

//         if (GCollision_GJK::GJKTest(ShapeSphere, TransSphere, ShapeBoxA, TBoxShapeA, &Tdraw))
//         {
//         
//         }

        GCollisionContact TContact;
        TContact.Clear();

        FColor TColor = FColor::Yellow;

        if( GCollision_Box::Box_Box_Contact_PhysX(ShapeBoxA, TBoxShapeA, ShapeBoxB, TBoxShapeB, &TContact ) != 0 )
        {
            GPhyscsUtils::DrawContact(TContact, &Tdraw, GColor::White());
            TColor = FColor::Red;
        }

        UKismetSystemLibrary::DrawDebugCoordinateSystem( GetWorld(), BoxCenterB, BoxRotB, 50. );

        UKismetSystemLibrary::DrawDebugBox(GetWorld(), BoxCenterA, BoxHalfSizeA, FColor::Yellow, BoxRotA);
        UKismetSystemLibrary::DrawDebugBox(GetWorld(), BoxCenterB, BoxHalfSizeB, TColor, BoxRotB);
    }
    if( CapsuleShow )
    {
        FQuat TRotation = FQuat::FindBetween(FVector(0, 0, 1), CapsuleDir);

       // UKismetSystemLibrary::DrawDebugCapsule(GetWorld(), CapsuleCenter, CapsuleHalfHeight + CapsuleRadius, CapsuleRadius, TRotation.Rotator(), FColor::Yellow);

        GShapeCapsule ShapeCapusle( GUtility::Unit_U_to_G(CapsuleHalfHeight), GUtility::Unit_U_to_G(CapsuleRadius) );

        GTransform_QT TransCapsule(GUtility::U_to_G(TRotation), GUtility::Unit_U_to_G(CapsuleCenter));

        Tdraw.DrawCapsule( TransCapsule, GUtility::Unit_U_to_G(CapsuleRadius), GUtility::Unit_U_to_G(CapsuleHalfHeight), FColor::Yellow.DWColor(), 24 );

        FColor TColor = FColor::Yellow;

        if (GCollision_GJK::GJKTest(ShapeSphere, TransSphere, ShapeCapusle, TransCapsule, &Tdraw))
        {
            TColor = FColor::Red;
        }

        UKismetSystemLibrary::DrawDebugSphere(GetWorld(), GetActorLocation(), TestSphereRadius, 20, TColor);
    }

    if( SphereShow )
    {
        GShapeSphere ShapeSA(GUtility::Unit_U_to_G(SphereRadiusA));
        GShapeSphere ShapeSB(GUtility::Unit_U_to_G(SphereRadiusB));

        FVector SCenterA = GetActorLocation();

        GTransform_QT TransA(GQuaternion::Identity(), GUtility::Unit_U_to_G(SCenterA));
        GTransform_QT TransB(GQuaternion::Identity(), GUtility::Unit_U_to_G(SphereCenterB));

        GCollisionContact TContact;
        TContact.Clear();

        FColor TColor = FColor::Yellow;
        if (GCollision_Sphere::Sphere_Sphere_Contact(ShapeSA, TransA, ShapeSB, TransB, &TContact) != 0)
        {
            GPhyscsUtils::DrawContact(TContact, &Tdraw, GColor::Yellow());

            TColor = FColor::Red;
        }

        UKismetSystemLibrary::DrawDebugSphere(GetWorld(), SCenterA, SphereRadiusA, 16, FColor::Yellow);
        UKismetSystemLibrary::DrawDebugSphere(GetWorld(), SphereCenterB, SphereRadiusB, 16, TColor);
    }

    if( TriangleShow )
    {
        FColor TriangleColor = FColor::Yellow;
        UKismetSystemLibrary::DrawDebugLine(GetWorld(), Triangle_Pos + Triangle_p0, Triangle_Pos + Triangle_p1, TriangleColor);
        UKismetSystemLibrary::DrawDebugLine(GetWorld(), Triangle_Pos + Triangle_p1, Triangle_Pos + Triangle_p2, TriangleColor);
        UKismetSystemLibrary::DrawDebugLine(GetWorld(), Triangle_Pos + Triangle_p0, Triangle_Pos + Triangle_p2, TriangleColor);
    }

    if( CovexHullShow )
    {
        if( pBuilder != nullptr )
        {
           // pBuilder->Draw( &Tdraw, GTransform_QT(GUtility::Unit_U_to_G(CovexHullCenter)), GColor::Yellow() );

        }
        
        if( pConvexHullA != nullptr )
        {
            pConvexHullA->Draw( &Tdraw, GTransform_QT(GUtility::U_to_G( ConvexRotA.Quaternion()),GUtility::Unit_U_to_G(CovexHullCenterA)), GColor::Yellow() );
        }

        if (pConvexHullB != nullptr)
        {
            pConvexHullB->Draw(&Tdraw, GTransform_QT(GUtility::U_to_G( ConvexRotB.Quaternion()), GUtility::Unit_U_to_G(CovexHullCenterB)), GColor::Yellow());
        }

        if( pConvexHullA != nullptr && pConvexHullB != nullptr)
        {
            GConvexHull TResult;

            pBuilder->BuildMinkowskiSum(
                *pConvexHullA, GTransform_QT(GUtility::U_to_G( ConvexRotA.Quaternion()), GUtility::Unit_U_to_G(CovexHullCenterA)),
                *pConvexHullB, GTransform_QT(GUtility::U_to_G( ConvexRotB.Quaternion()), GUtility::Unit_U_to_G(CovexHullCenterB)),
                TResult, false);

            TResult.Draw(&Tdraw, GTransform_QT::Identity(), GColor::Yellow());
        }
    }

    if(TestSphere_Box)
    {
        GShapeBox ShapeBoxA(GUtility::Unit_U_to_G(BoxHalfSizeA));
        FVector BoxCenterA = GetActorLocation();
        FRotator BoxRotA = GetActorRotation();
        GTransform_QT TBoxShapeA(GUtility::U_to_G(BoxRotA.Quaternion()), GUtility::Unit_U_to_G(BoxCenterA));

        GShapeSphere ShapeSA(GUtility::Unit_U_to_G(SphereRadiusB));
        GTransform_QT TransA(GQuaternion::Identity(), GUtility::Unit_U_to_G(SphereCenterB));


        GCollisionContact TContact;
        TContact.Clear();

        FColor TColor = FColor::Yellow;
        if (GCollision_Sphere::Sphere_Box_Contact(ShapeSA, TransA, ShapeBoxA, TBoxShapeA, &TContact) != 0)
        {

            for (int i = 0; i < TContact.GetPointCount(); ++i)
            {
                const GManifoldPoint& TMn = TContact.m_Point[i];

                FVector VPos = GUtility::Unit_G_to_U(TMn.m_PosWorld);
                FVector VNor = GUtility::Unit_G_to_U(TMn.m_NormalOnB);

                FVector Vdes = VPos + VNor * GUtility::G_to_U(TMn.m_depth);

                UKismetSystemLibrary::DrawDebugSphere(GetWorld(), VPos, 3.f, 12, FColor::White);
                UKismetSystemLibrary::DrawDebugSphere(GetWorld(), Vdes, 1.f, 12, FColor::White);
                UKismetSystemLibrary::DrawDebugLine(GetWorld(), VPos, Vdes, FColor::White);

            }
            TColor = FColor::Red;
        }

        UKismetSystemLibrary::DrawDebugCoordinateSystem(GetWorld(), BoxCenterB, BoxRotB, 50.);

        UKismetSystemLibrary::DrawDebugBox(GetWorld(), BoxCenterA, BoxHalfSizeA, TColor, BoxRotA);
        UKismetSystemLibrary::DrawDebugSphere(GetWorld(), SphereCenterB, SphereRadiusB, 16, TColor);


    }


//     if (GCollision_GJK::GJKTest(ShapeSphere, TransShapeA, ShapeBox, TransShapeB, &Tdraw ))
//     {
//       //  UKismetSystemLibrary::DrawDebugSphere(GetWorld(), VT, TestSphereRadius + 1.f, 28, FColor::Red);
//      //   UKismetSystemLibrary::DrawDebugLine(GetWorld(), VT, VT + OutNormal * 100.f, FColor::Black);
//     }


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

