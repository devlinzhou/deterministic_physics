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

DECLARE_STATS_GROUP(TEXT("GPhysics)"), STATGROUP_GPhysics, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("GPhysics ActorTick"),              STAT_AGPhysicsActor_Tick,               STATGROUP_GPhysics);
DECLARE_CYCLE_STAT(TEXT("GPhysics Total"),                  STAT_GPysics_Total,                     STATGROUP_GPhysics);
DECLARE_CYCLE_STAT(TEXT("GPhysics PreTick"),                STAT_GPysics_PreTick,                   STATGROUP_GPhysics);
DECLARE_CYCLE_STAT(TEXT("GPhysics Simulate"),               STAT_GPysics_Simulate,                  STATGROUP_GPhysics);
DECLARE_CYCLE_STAT(TEXT("GPhysics CollisionBroadPhase"),    STAT_GPysics_CollisionBroadPhase,       STATGROUP_GPhysics);
DECLARE_CYCLE_STAT(TEXT("GPhysics CollisionNarrowPhase"),   STAT_GPysics_CollisionNarrowPhase,      STATGROUP_GPhysics);
DECLARE_CYCLE_STAT(TEXT("GPhysics SolveContactConstraint"), STAT_GPysics_SolveContactConstraint,    STATGROUP_GPhysics);
DECLARE_CYCLE_STAT(TEXT("GPhysics UpdateSceneGrid"),        STAT_GPysics_UpdateSceneGrid,           STATGROUP_GPhysics);
DECLARE_CYCLE_STAT(TEXT("GPhysics PostTick"),               STAT_GPysics_PostTick,                  STATGROUP_GPhysics);
DECLARE_CYCLE_STAT(TEXT("GPhysics DebugDraw"),              STAT_GPysics_DebugDraw,                 STATGROUP_GPhysics);


void AGPhysicsActor::GPysics_PreTick()
{
    SCOPE_CYCLE_COUNTER(STAT_GPysics_PreTick);
    m_PhysicsWorld.PreTick();
}

void AGPhysicsActor::GPysics_Simulate(float DeltaTime)
{
    SCOPE_CYCLE_COUNTER(STAT_GPysics_Simulate);
    m_PhysicsWorld.Simulate(GMath::FromFloat(DeltaTime));
}

void AGPhysicsActor::GPysics_CollisionBroadPhase()
{
    SCOPE_CYCLE_COUNTER(STAT_GPysics_CollisionBroadPhase);
    m_PhysicsWorld.CollisionBroadPhase();
}

void AGPhysicsActor::GPysics_CollisionNarrowPhase()
{
    SCOPE_CYCLE_COUNTER(STAT_GPysics_CollisionNarrowPhase);
    m_PhysicsWorld.CollisionNarrowPhase();
}

void AGPhysicsActor::GPysics_SolveContactConstraint()
{
    SCOPE_CYCLE_COUNTER(STAT_GPysics_SolveContactConstraint);
    m_PhysicsWorld.SolveContactConstraint();
}

void AGPhysicsActor::GPysics_UpdateSceneGrid()
{
    SCOPE_CYCLE_COUNTER(STAT_GPysics_UpdateSceneGrid);
    m_PhysicsWorld.UpdateSceneGrid();
}

void AGPhysicsActor::GPysics_PostTick()
{
    SCOPE_CYCLE_COUNTER(STAT_GPysics_PostTick);
    m_PhysicsWorld.PostTick();
}

void AGPhysicsActor::GPysics_DebugDraw()
{
    SCOPE_CYCLE_COUNTER(STAT_GPysics_DebugDraw);
    GPhysicsDraw TDraw(GetWorld());

    m_PhysicsWorld.DebugDraw(&TDraw, (uint32_t)DrawMask);
}
// Called every frame
void AGPhysicsActor::Tick(float DeltaTime)
{
    SCOPE_CYCLE_COUNTER(STAT_AGPhysicsActor_Tick);
	Super::Tick(DeltaTime);
    {
        if (!UGameplayStatics::IsGamePaused(GetWorld()))
        {   
            SCOPE_CYCLE_COUNTER(STAT_GPysics_Total);
            GPysics_PreTick();
            GPysics_Simulate(DeltaTime);
            GPysics_CollisionBroadPhase();
            GPysics_CollisionNarrowPhase();
            GPysics_SolveContactConstraint();
            GPysics_UpdateSceneGrid();
            GPysics_PostTick();
        }

        GPysics_DebugDraw();

        if (GEngine)
        {
            float TS = GMath::ToFloat(m_PhysicsWorld.GetTotalEnergy());
            FVector LM = GUtility::G_to_U( m_PhysicsWorld.GetTotalLinearMomentum());
            FVector AM = GUtility::G_to_U( m_PhysicsWorld.GetTotalAngularMomentum());


            GEngine->AddOnScreenDebugMessage(1, 15.0f, FColor::Yellow, FString::Printf(TEXT("total energy: %f "), TS));

            GEngine->AddOnScreenDebugMessage(2, 15.0f, FColor::Yellow, 
                FString::Printf(TEXT("Linear Momentum : %.3f, %.3f, %.3f "), LM.X, LM.Y, LM.Z));

            GEngine->AddOnScreenDebugMessage(3, 15.0f, FColor::Yellow,
                FString::Printf(TEXT("Angular Momentum : %.3f, %.3f, %.3f "), AM.X, AM.Y, AM.Z));

        }
    }
}

GRigidBody* AGPhysicsActor::CreateSimpleRigidBody( const GTransform_QT& Trans, GVector3 Halfsize, EShape TShape )
{
    GRigidBody* pBody = new GRigidBody(m_PhysicsWorld.GetNewCObjectId(), TShape);

    pBody->m_Shape.SetHalfExtern(Halfsize);
    pBody->UpdateLocalBox();
    pBody->CalculateInertiaTensor();
    pBody->m_Transform = Trans;

    m_PhysicsWorld.AddCollisionObject(pBody);

    return pBody;
}

GRigidBody* AGPhysicsActor::CreateStaticPlane(const GTransform_QT& Trans )
{
    GRigidBody* pBody = new GRigidBody(m_PhysicsWorld.GetNewCObjectId(), EShape::EShape_Plane);
    pBody->UpdateLocalBox();
 
    m_PhysicsWorld.AddStaticLargeObj(pBody);
    pBody->m_Transform = Trans;
    pBody->m_bDynamic = false;
    return pBody;
}
