// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GPhysicsCollistionActor.generated.h"

UCLASS()
class UNREALDEMO_API AGPhysicsCollistionActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AGPhysicsCollistionActor();
    bool ShouldTickIfViewportsOnly() const override { return true; }
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

    UPROPERTY(EditAnywhere, Category = "Sphere")
        FVector         SphereCenter = FVector(400, 200, 90);
    UPROPERTY(EditAnywhere, Category = "Sphere")
        float           SphereRadius = 100.f;

    UPROPERTY(EditAnywhere, Category = "Triangle")
        FVector        Triangle_Pos = FVector(111, 0, 0);
    UPROPERTY(EditAnywhere, Category = "Triangle")
        FVector        Triangle_p0 = FVector(10, 0, 0);
    UPROPERTY(EditAnywhere, Category = "Triangle")
        FVector        Triangle_p1 = FVector(0, 10, 0);
    UPROPERTY(EditAnywhere, Category = "Triangle")
        FVector        Triangle_p2 = FVector(10, 10, 0);

    UPROPERTY(EditAnywhere, Category = "Box")
        FVector        BoxCenter = FVector(0, 0, 200);
    UPROPERTY(EditAnywhere, Category = "Box")
        FVector        BoxHalfSize = FVector(100, 200, 90);
    UPROPERTY(EditAnywhere, Category = "Box")
        FRotator       BoxRot       = FRotator(0, 0, 0);

    UPROPERTY(EditAnywhere, Category = "Capsule")
        FVector         CapsuleCenter = FVector(100, 200, 90);
    UPROPERTY(EditAnywhere, Category = "Capsule")
        FVector         CapsuleDir = FVector(0, 0, 1);
    UPROPERTY(EditAnywhere, Category = "Capsule")
        float           CapsuleRadius = 20.f;
    UPROPERTY(EditAnywhere, Category = "Capsule")
        float           CapsuleHalfHeight = 20.f;



    UPROPERTY(EditAnywhere, Category = "TestSphere")
        float           TestSphereRadius = 50.f;

};
