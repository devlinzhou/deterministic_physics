// Fill out your copyright notice in the Description page of Project Settings.


#include "GPhysicsActor.h"
#include "glacier_rigid_static.h"
#include "glacier_rigid_dynamic.h"
#include "glacier_debug_draw.h"
#include "GUnrealUtility.h"


// Sets default values
AGPhysicsActor::AGPhysicsActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AGPhysicsActor::BeginPlay()
{
	Super::BeginPlay();

    m_PhysicsWorld.Init();

    {
        GStaticRigid* pFloor = new GStaticRigid(m_PhysicsWorld.CollisionId++, EShape_Box);
        pFloor->m_Shape.SetHalfExtern(GVector3(GMath::Three(), GMath::Three(), GMath::Half()));
        pFloor->UpdateLocalBox();
        m_PhysicsWorld.AddCollisionObject(pFloor);
    }

    {
    
        CreateRigidBox( GVector3(0,0,10), GVector3(1,1,1) );
    
    }
	
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

GDynamicRigid* AGPhysicsActor::CreateRigidBox(  GVector3 VPos, GVector3 Halfsize )
{
    GDynamicRigid* pBox = new GDynamicRigid(m_PhysicsWorld.CollisionId++, EShape_Box);
    pBox->m_Shape.SetHalfExtern(GVector3(GMath::Half(), GMath::Half(), GMath::Half()));
    pBox->UpdateLocalBox();
    m_PhysicsWorld.AddCollisionObject(pBox);
    pBox->m_Transform.m_Translation = VPos;
    pBox->m_VelocityMax = GMath::Makef32(1,0,1);

   // pFloor->m_Gravity = 

    return pBox;

}