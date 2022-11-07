// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "glacier_physics_world.h"
#include "glacier_collision_shape.h"
#include "GPhysicsActor.generated.h"


class GStaticRigid;
class GRigidBody;

UENUM(meta = (Bitflags ))
enum class EGDrawMask : uint32
{
    UGPDraw_Shape      ,
    UGPDraw_LocalBox    ,
    UGPDraw_WorldBox    ,
    UGPDraw_CeilBox     ,
    UGPDraw_Contact     ,
};
ENUM_CLASS_FLAGS(EGDrawMask);

UCLASS()
class UNREALDEMO_API AGPhysicsActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AGPhysicsActor();

    bool ShouldTickIfViewportsOnly() const override
    {
        return true;
    }

   static AGPhysicsActor* FindScenePhysics( class UWorld* );

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

    GRigidBody* CreateStaticRigidBody( const GTransform_QT& Trans, GVector3 Halfsize, EShape TShape );
    GRigidBody* CreateDynamicRigidBody( const GTransform_QT& Trans, GVector3 Halfsize, EShape TShape );

    GPhysicsWorld m_PhysicsWorld;

    UPROPERTY(EditAnywhere,meta = (Bitmask, BitmaskEnum = "EGDrawMask"))
    int32   DrawMask;

};
