// Fill out your copyright notice in the Description page of Project Settings.


#include "GPhysicsActor.h"

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
}

