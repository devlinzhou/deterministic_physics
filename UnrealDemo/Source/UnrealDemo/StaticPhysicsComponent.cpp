// Fill out your copyright notice in the Description page of Project Settings.


#include "StaticPhysicsComponent.h"
#include "GPhysicsActor.h"
#include "glacier_rigid_body.h"
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
        switch ( RigidShape )
        {
        case UGShape::UGShape_Sphere :
        case UGShape::UGShape_Box :
        {
            m_pRigid = PPhysics->CreateStaticRigidBody(
                GUtility::U_to_G(GetOwner()->GetTransform()),
                GVector3(GUtility::Unit_U_to_G(VHalfSize)),
                RigidShape ==  UGShape::UGShape_Sphere ?  EShape::EShape_Sphere : EShape::EShape_Box );
        }
        break;
        case UGShape::UGShape_Plane :
        {
            m_pRigid = PPhysics->CreateStaticPlane( GUtility::U_to_G(GetOwner()->GetTransform()));
        }
        break;

        default:
            break;
        }

    }
	
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

