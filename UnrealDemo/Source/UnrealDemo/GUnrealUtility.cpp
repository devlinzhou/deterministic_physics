// Fill out your copyright notice in the Description page of Project Settings.

#include "GUnrealUtility.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"


void GPhysicsDraw::DrawLine(const GVector3& V0, const GVector3& V1, GColor uColor)
{
    FVector TV0 = GUtility::Unit_G_to_U(V0);
    FVector TV1 = GUtility::Unit_G_to_U(V1);

    UKismetSystemLibrary::DrawDebugLine(m_World, TV0, TV1, FColor(uColor.RawValue));
}

