// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "rigid_sphere.generated.h"

UCLASS()
class UNREALDEMO_API Arigid_sphere : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	Arigid_sphere();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

    UPROPERTY(EditAnywhere)
    class AGPhysicsActor* PPhysics;

    UPROPERTY(EditAnywhere)
    float Radius;

};
