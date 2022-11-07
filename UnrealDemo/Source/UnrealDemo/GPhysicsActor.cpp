// Fill out your copyright notice in the Description page of Project Settings.


#include "GPhysicsActor.h"
#include "glacier_rigid_body.h"
#include "glacier_debug_draw.h"
#include "GUnrealUtility.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"

// Sets default values
AGPhysicsActor::AGPhysicsActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = true;
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

    if(!UGameplayStatics::IsGamePaused( GetWorld()))
        m_PhysicsWorld.Tick( GMath::FromFloat(DeltaTime) );

    GPhysicsDraw TDraw(GetWorld());

    m_PhysicsWorld.DebugDraw( &TDraw, (uint32_t)DrawMask );
}

GRigidBody* AGPhysicsActor::CreateStaticRigidBody(const GTransform_QT& Trans, GVector3 Halfsize, EShape TShape)
{
    GRigidBody* pBody = new GRigidBody(m_PhysicsWorld.CollisionId++, TShape);

    pBody->m_Shape.SetHalfExtern(Halfsize);
    pBody->UpdateLocalBox();
    m_PhysicsWorld.AddCollisionObject(pBody);
    pBody->m_Transform = Trans;
    pBody->m_bDynamic = false;
    return pBody;
}

GRigidBody* AGPhysicsActor::CreateDynamicRigidBody( const GTransform_QT& Trans, GVector3 Halfsize, EShape TShape)
{
    GRigidBody* pBody = new GRigidBody(m_PhysicsWorld.CollisionId++, TShape);

    pBody->m_Shape.SetHalfExtern(Halfsize);

    pBody->UpdateLocalBox();
    m_PhysicsWorld.AddCollisionObject(pBody);
    pBody->m_Transform = Trans;
    pBody->m_VelocityMax = GMath::Makef32(10, 0, 1);
     pBody->m_bDynamic = true;
    return pBody;
}