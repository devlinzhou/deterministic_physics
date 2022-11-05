// Fill out your copyright notice in the Description page of Project Settings.


#include "GPhysicsActor.h"
#include "glacier_rigid_static.h"
#include "glacier_rigid_dynamic.h"
#include "glacier_debug_draw.h"
#include "GUnrealUtility.h"
#include "EngineUtils.h"

// Sets default values
AGPhysicsActor::AGPhysicsActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

AGPhysicsActor* AGPhysicsActor::FindScenePhysics( UWorld* pWorld)
{
    AGPhysicsActor* pActor = nullptr;
    for (TActorIterator<AGPhysicsActor> Iter(pWorld); Iter; ++Iter)
    {
        pActor = *Iter;
        break;
    }
    return pActor;
}

// Called when the game starts or when spawned
void AGPhysicsActor::BeginPlay()
{
	Super::BeginPlay();

    m_PhysicsWorld.Init();
}


void AGPhysicsActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);

    m_PhysicsWorld.UnInit();
}

// Called every frame
void AGPhysicsActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

    m_PhysicsWorld.Tick( GMath::FromFloat(DeltaTime) );

    GPhysicsDraw TDraw(GetWorld());

    m_PhysicsWorld.DebugDraw( &TDraw );
}

GStaticRigid* AGPhysicsActor::CreateStaticRigidBody(const GTransform_QT& Trans, GVector3 Halfsize, EShape TShape)
{
    GStaticRigid* pBody = new GStaticRigid(m_PhysicsWorld.CollisionId++, TShape);

    pBody->m_Shape.SetHalfExtern(Halfsize);
    pBody->UpdateLocalBox();
    m_PhysicsWorld.AddCollisionObject(pBody);
    pBody->m_Transform = Trans;

    return pBody;
}

GDynamicRigid* AGPhysicsActor::CreateDynamicRigidBody( const GTransform_QT& Trans, GVector3 Halfsize, EShape TShape)
{
    GDynamicRigid* pBody = new GDynamicRigid(m_PhysicsWorld.CollisionId++, TShape);

    pBody->m_Shape.SetHalfExtern(Halfsize);

    pBody->UpdateLocalBox();
    m_PhysicsWorld.AddCollisionObject(pBody);
    pBody->m_Transform = Trans;
    pBody->m_VelocityMax = GMath::Makef32(10, 0, 1);

    // pFloor->m_Gravity = 

    return pBody;
}