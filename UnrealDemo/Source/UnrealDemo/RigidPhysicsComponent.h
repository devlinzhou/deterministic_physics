// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RigidPhysicsComponent.generated.h"

UENUM()
enum class UGShape : uint8
{
    UGShape_Sphere,
    UGShape_Box,
    UGShape_Plane,

};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UNREALDEMO_API URigidPhysicsComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	URigidPhysicsComponent();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    UPROPERTY(EditAnywhere)
        bool        Dynamic = false;
    UPROPERTY(EditAnywhere)
        bool        ModifySize = false;
    UPROPERTY(EditAnywhere)
        UGShape     RigidShape = UGShape::UGShape_Sphere;
    UPROPERTY(EditAnywhere)
        FVector     VHalfSize = FVector(50,50,50);
    UPROPERTY(EditAnywhere)
        FVector     Gravity = FVector(0,0,0);
    UPROPERTY(EditAnywhere)
        float       MaxLinearVelocity = 2000.f;
    UPROPERTY(EditAnywhere)
        float       MaxAngularVelocity = 100.f;

    class GRigidBody* m_pRigid = nullptr;

};
