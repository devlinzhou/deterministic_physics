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


    GPhysicsDraw       Tdraw(GetWorld());

    GShapeBox BoxShapeA(GUtility::Unit_U_to_G(BoxHalfSizeA));
    GShapeBox BoxShapeB(GUtility::Unit_U_to_G(BoxHalfSizeB));

    FVector     AShapePos = GetActorLocation();
    FRotator    BoxRotA     = GetActorRotation();

    GTransform_QT BoxTransA(GUtility::U_to_G(BoxRotA.Quaternion()), GUtility::Unit_U_to_G(AShapePos));
    GTransform_QT BoxTransB(GUtility::U_to_G(BoxRotB.Quaternion()), GUtility::Unit_U_to_G(BoxCenterB));

    GShapeSphere SphereShapeA(GUtility::Unit_U_to_G(SphereRadiusA));
    GShapeSphere SphereShapeB(GUtility::Unit_U_to_G(SphereRadiusB));

    GTransform_QT SphereTransA(GQuaternion::Identity(), GUtility::Unit_U_to_G(AShapePos));
    GTransform_QT SphereTransB(GQuaternion::Identity(), GUtility::Unit_U_to_G(SphereCenterB));

    GShapeCapsule ShapeCapusle(GUtility::Unit_U_to_G(CapsuleHalfHeight), GUtility::Unit_U_to_G(CapsuleRadius));
    GTransform_QT TransCapsule(GUtility::U_to_G(FQuat::FindBetween(FVector(0, 0, 1), CapsuleDir)), GUtility::Unit_U_to_G(CapsuleCenter));

    if( BoxShow )
    {
//         if (GCollision_GJK::GJKTest(ShapeSphere, TransSphere, ShapeBoxA, TBoxShapeA, &Tdraw))
//         {
//         
//         }

        GCollisionContact TContact;
        TContact.Clear();

        GColor TColor = GColor::Yellow();

        bool bswap = false;

        if( GCollision_Box::Box_Box_Contact_PX(BoxShapeA, BoxTransA, BoxShapeB, BoxTransB, &TContact, bswap ) != 0 )
        {
            GPhyscsUtils::DrawContact(TContact, &Tdraw, GColor::White());
            TColor = GColor::Red();
        }

        GPhyscsUtils::DrawBox(BoxTransA, BoxShapeA, &Tdraw, GColor::Yellow() );
        GPhyscsUtils::DrawBox(BoxTransB, BoxShapeB, &Tdraw, TColor);
        GPhyscsUtils::DrawCoordinateSystem(&Tdraw, BoxTransB, GMath::Half() );
    }
    if( CapsuleShow )
    {
        GColor TColor = GColor::Yellow();
        if (GCollision_GJK::GJKTest(SphereShapeA, SphereTransA, ShapeCapusle, TransCapsule, &Tdraw))
        {
            TColor = GColor::Red();
        }

        GPhyscsUtils::DrawCapsule( TransCapsule, ShapeCapusle, &Tdraw, TColor );
        GPhyscsUtils::DrawSphere( SphereTransA, SphereShapeA, &Tdraw, TColor);
    }

    if( SphereShow )
    {
        GCollisionContact TContact;
        TContact.Clear();

        GColor TColor = GColor::Yellow();
        if (GCollision_Sphere::Sphere_Sphere_Contact(SphereShapeA, SphereTransA, SphereShapeB, SphereTransB, &TContact) != 0)
        {
            GPhyscsUtils::DrawContact(TContact, &Tdraw, GColor::White());
            TColor = GColor::Red();
        }

        GPhyscsUtils::DrawSphere(SphereTransA, SphereShapeA, &Tdraw, GColor::Yellow());
        GPhyscsUtils::DrawSphere(SphereTransB, SphereShapeB, &Tdraw, TColor);
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
        GCollisionContact TContact;
        TContact.Clear();

        GColor TColor = GColor::Yellow();
        if (GCollision_Sphere::Sphere_Box_Contact(SphereShapeA, SphereTransA, BoxShapeA, BoxTransA, &TContact) != 0)
        {
            GPhyscsUtils::DrawContact(TContact, &Tdraw, GColor::White());
            TColor = GColor::Red();
        }

        GPhyscsUtils::DrawBox(BoxTransA, BoxShapeA, &Tdraw, GColor::Yellow() );
        GPhyscsUtils::DrawSphere(SphereTransB, SphereShapeB, &Tdraw, TColor);
        GPhyscsUtils::DrawCoordinateSystem(&Tdraw, BoxTransB, GMath::Half() );
    }


}

