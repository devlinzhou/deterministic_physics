// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "glacier_vector.h"
#include "glacier_quaternion.h"
#include "glacier_transform_qt.h"
#include "glacier_debug_draw.h"

class GUtility
{
public:

    static constexpr float fUTG = 0.01f;
    static constexpr float fGTU = 100.f;

    static inline f32           U_to_G(float value)                 {return GMath::FromFloat(value );}
    static inline GVector3      U_to_G(const FVector& value)        {return GVector3( GMath::FromFloat(value.X ),GMath::FromFloat(value.Y ),GMath::FromFloat(value.Z )); }
    static inline GQuaternion   U_to_G(const FQuat& value)          {return GQuaternion(GMath::FromFloat(value.X), GMath::FromFloat(value.Y), GMath::FromFloat(value.Z),GMath::FromFloat(value.W)); }
    static inline GTransform_QT U_to_G(const FTransform& value)     {return GTransform_QT(U_to_G(value.GetRotation()), Unit_U_to_G(value.GetTranslation())); }
    
    static inline float         G_to_U(f32 value)                   {return GMath::ToFloat(value); }
    static inline FVector       G_to_U(const GVector3& value)       {return FVector(GMath::ToFloat(value.x) , GMath::ToFloat(value.y), GMath::ToFloat(value.z)); }
    static inline FQuat         G_to_U(const GQuaternion& value)    {return FQuat(GMath::ToFloat(value.x), GMath::ToFloat(value.y), GMath::ToFloat(value.z), GMath::ToFloat(value.w)); }
    static inline FTransform    G_to_U(const GTransform_QT& value)  {return FTransform(G_to_U(value.m_Rot), Unit_G_to_U(value.m_Pos)); }

    static inline f32           Unit_U_to_G(float value)            {return U_to_G(value * fUTG);}
    static inline GVector3      Unit_U_to_G(const FVector& value)   {return U_to_G(value * fUTG);}
    static inline float         Unit_G_to_U(f32 value)              {return G_to_U(value) * fGTU;}
    static inline FVector       Unit_G_to_U(const GVector3& value)  {return G_to_U(value) * fGTU;}
};


class GPhysicsDraw : public  IGlacierDraw
{
public:
    GPhysicsDraw(UWorld* pWorld)
    {
        m_World = pWorld;
    }

    virtual void DrawLine(const GVector3& V0, const GVector3& V1, GColor uColor);

    UWorld* m_World = nullptr;
};

