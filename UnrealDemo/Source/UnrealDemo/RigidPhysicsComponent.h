// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "StaticPhysicsComponent.h"
#include "RigidPhysicsComponent.generated.h"


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
        UGShape RigidShape;
    UPROPERTY(EditAnywhere)
        FVector VHalfSize;
    UPROPERTY(EditAnywhere)
        FVector    Gravity;
    UPROPERTY(EditAnywhere)
        float  MaxVelocity;

    class GDynamicRigid* m_pRigid = nullptr;

};
