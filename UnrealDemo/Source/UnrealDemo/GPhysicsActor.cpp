// Fill out your copyright notice in the Description page of Project Settings.


#include "GPhysicsActor.h"
#include "glacier_rigid_static.h"
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

   GStaticRigid* pFloor = new GStaticRigid( 0, EShape_Box );

   pFloor->m_Shape.SetHalfExtern( GVector3( GMath::Three(), GMath::Three() , GMath::Half() ));

   m_PhysicsWorld.AddCollisionObject( pFloor );
	
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

void AGPhysicsActor::CreateBox( )
{
    //new G


}