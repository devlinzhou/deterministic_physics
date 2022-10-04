// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "glacier_vector.h"


class GUtility
{
public:

    static constexpr float fUTG = 0.01f;
    static constexpr float fGTU = 100.f;

    static inline f32           U_to_G(float value)                 {return f32(value );}
    static inline GVector3      U_to_G(const FVector& value)        {return GVector3( f32(value.X ),f32(value.Y ),f32(value.Z )); }
    static inline GQuaternion   U_to_G(const FQuat& value)          {return GQuaternion(f32(value.X), f32(value.Y), f32(value.Z),f32(value.W)); }
    static inline float         G_to_U(f32 value)                   {return GMath::ToFloat(value); }
    static inline FVector       G_to_U(const GVector3& value)       {return FVector(float(value.x) , float(value.y), float(value.z)); }
    static inline FQuat         G_to_U(const GQuaternion& value)    {return FQuat(float(value.x), float(value.y), float(value.z), float(value.w)); }
    static inline f32           Unit_U_to_G(float value)            {return U_to_G(value * fUTG);}
    static inline GVector3      Unit_U_to_G(const FVector& value)   {return U_to_G(value * fUTG);}
    static inline float         Unit_G_to_U(f32 value)              {return G_to_U(value) * fGTU;}
    static inline FVector       Unit_G_to_U(const GVector3& value)  {return G_to_U(value) * fGTU;}

};


