// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "glacier_collision_shape.h"
#include "StaticPhysicsComponent.generated.h"

UENUM()
enum class UGShape : uint8
{
    UGShape_Sphere,
    UGShape_Box,
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UNREALDEMO_API UStaticPhysicsComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UStaticPhysicsComponent();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    UPROPERTY(EditAnywhere)
        UGShape RigidShape;

    UPROPERTY(EditAnywhere)
        FVector VHalfSize;

	class GDynamicRigid* m_pRigid = nullptr;
};
