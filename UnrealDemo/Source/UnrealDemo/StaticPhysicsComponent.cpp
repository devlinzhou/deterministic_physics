// Fill out your copyright notice in the Description page of Project Settings.


#include "StaticPhysicsComponent.h"
#include "GPhysicsActor.h"
#include "glacier_rigid_static.h"
#include "GUnrealUtility.h"

// Sets default values for this component's properties
UStaticPhysicsComponent::UStaticPhysicsComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UStaticPhysicsComponent::BeginPlay()
{
	Super::BeginPlay();

    AGPhysicsActor* PPhysics = AGPhysicsActor::FindScenePhysics(GetWorld());

    if( PPhysics != nullptr )
    {
        EShape TShape = EShape_ConvexBase;

        if( RigidShape == UGShape::UGShape_Sphere)
            TShape = EShape_Sphere;
        else if( RigidShape == UGShape::UGShape_Box)
            TShape = EShape_Box;

        if(TShape != EShape_ConvexBase) 
        {
            m_pRigid = PPhysics->CreateStaticRigidBody( 
                GUtility::U_to_G(GetOwner()->GetTransform()),
                GVector3(GUtility::Unit_U_to_G(VHalfSize)), TShape);
        }

    }

	// ...
	
}


// Called every frame
void UStaticPhysicsComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if( m_pRigid != nullptr )
    {
        m_pRigid->m_Transform = GUtility::U_to_G( GetOwner()->GetTransform() );

        m_pRigid->NeedUpdate();
    }
	// ...
}

