// Fill out your copyright notice in the Description page of Project Settings.


#include "RigidPhysicsComponent.h"
#include "GPhysicsActor.h"
#include "glacier_rigid_body.h"
#include "GUnrealUtility.h"


    
// Sets default values for this component's properties
URigidPhysicsComponent::URigidPhysicsComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void URigidPhysicsComponent::BeginPlay()
{
	Super::BeginPlay();

    AGPhysicsActor* PPhysics = AGPhysicsActor::FindScenePhysics(GetWorld());

    if (PPhysics != nullptr)
    {
        switch (RigidShape)
        {
        case UGShape::UGShape_Sphere:
        case UGShape::UGShape_Box:
        {
            m_pRigid = PPhysics->CreateSimpleRigidBody(
                GUtility::U_to_G(GetOwner()->GetTransform()),
                GVector3(GUtility::Unit_U_to_G(VHalfSize)),
                RigidShape == UGShape::UGShape_Sphere ? EShape::EShape_Sphere : EShape::EShape_Box);

            m_pRigid->m_Gravity = GUtility::U_to_G(Gravity);
            m_pRigid->m_LinearVelocityMax = GUtility::U_to_G(MaxLinearVelocity);
           // m_pRigid->m_AngularVelocityMax = GUtility::U_to_G(MaxAngularVelocity);

            m_pRigid->m_AngularVelocityMax = GMath::Makef32(10,0,1);

            m_pRigid->m_LinearVelocity = GUtility::Unit_U_to_G(StartLinearVeloctiy);
            m_pRigid->m_AngularVelocity = GUtility::U_to_G(StartAngularVeloctiy);

			m_pRigid->m_LinearDamping	= GUtility::U_to_G(LinearDamping);
			m_pRigid->m_AngularDamping	= GUtility::U_to_G(AngularDamping);


            m_pRigid->m_bDynamic = Dynamic;

        }
        break;
        case UGShape::UGShape_Plane:
        {
            m_pRigid = PPhysics->CreateStaticPlane(GUtility::U_to_G(GetOwner()->GetTransform()));
            m_pRigid->m_bDynamic = false;
        }
        break;

        default:
            break;
        }
    }	
}


// Called every frame
void URigidPhysicsComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (m_pRigid != nullptr)
    {
        if (Dynamic)
        {
            GetOwner()->SetActorLocation(GUtility::Unit_G_to_U(m_pRigid->m_Transform.m_Pos));
            GetOwner()->SetActorRotation(GUtility::G_to_U(m_pRigid->m_Transform.m_Rot));
        }
        else
        {
            m_pRigid->m_Transform = GUtility::U_to_G(GetOwner()->GetTransform());

            m_pRigid->m_Transform.m_Rot.Normalize();
            m_pRigid->NeedUpdate();
        }

        if( ModifySize )
        {
            m_pRigid->m_Shape.HalfSize = GUtility::Unit_U_to_G(VHalfSize);
        }
    }
}

