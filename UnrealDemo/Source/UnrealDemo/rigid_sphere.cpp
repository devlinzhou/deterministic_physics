// Fill out your copyright notice in the Description page of Project Settings.


#include "rigid_sphere.h"

// Sets default values
Arigid_sphere::Arigid_sphere()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void Arigid_sphere::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void Arigid_sphere::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

