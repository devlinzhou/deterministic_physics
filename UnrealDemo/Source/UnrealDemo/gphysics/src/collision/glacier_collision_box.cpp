/* 
 * Copyright (C) 2022 zhou xuan, Email: zhouxuan6676@gmail.com
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at * 
 * http://www.apache.org/licenses/LICENSE-2.0 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and 
 * limitations under the License. 
 */

#include "glacier_collision_box.h"
#include "glacier_matrix.h"

static inline GVector3 Cross_X(f32 a, const GVector3& V)
{
    return GVector3(GMath::Zero(), -a * V.z, a * V.y);
}

static inline GVector3 Cross_Y(f32 a, const GVector3& V)
{
    return  GVector3(a * V.z, GMath::Zero(), -a * V.x);
}

static inline GVector3 Cross_Z(f32 a, const GVector3& V)
{
    return GVector3(-a * V.y, a * V.x, GMath::Zero());
}

static inline GVector3 Cross_AbsX(f32 a, const GVector3& V)
{
    return GVector3(GMath::Zero(), a * V.z, a * V.y);
}

static inline GVector3 Cross_AbsY(f32 a, const GVector3& V)
{
    return  GVector3(a * V.z, GMath::Zero(), a * V.x);
}

static inline GVector3 Cross_AbsZ(f32 a, const GVector3& V)
{
    return GVector3(a * V.y, a * V.x, GMath::Zero());
}

static inline f32 Dot_No_X( const GVector3& SatAxisLocal, const GVector3& HalfExten )
{
    return HalfExten.y * (SatAxisLocal.y) + HalfExten.z * (SatAxisLocal.z);
}

static inline f32 Dot_No_Y(const GVector3& SatAxisLocal, const GVector3& HalfExten)
{
    return HalfExten.x * (SatAxisLocal.x) + HalfExten.z * (SatAxisLocal.z);
}

static inline f32 Dot_No_Z(const GVector3& SatAxisLocal, const GVector3& HalfExten)
{
    return HalfExten.x * (SatAxisLocal.x) + HalfExten.y * (SatAxisLocal.y);
}

bool GCollision_Box::Box_Box(
    const GShapeBox&        ShapA,
    const GTransform_QT&    TransformA,
    const GShapeBox&        ShapB,
    const GTransform_QT&    TransformB,
    GVector3*               pOutPosition,
    GVector3*               pOutNormal)
{
    const GVector3& HalfA = ShapA.HalfExtern;
    const GVector3& HalfB = ShapB.HalfExtern;
    GTransform_QT   B_to_A      = TransformB * TransformA.GetInverse_fast();
    GMatrix3        M_B_To_A( B_to_A.m_Rotate );

    GVector3    BLAX      = M_B_To_A.GetRow(0);
    GVector3    BLAY      = M_B_To_A.GetRow(1);
    GVector3    BLAZ      = M_B_To_A.GetRow(2);
    GVector3&   VDisBA    = B_to_A.m_Translation;

    GVector3 VAxis_Abs0 = GVector3(GMath::Abs(BLAX.x), GMath::Abs(BLAY.x), GMath::Abs(BLAZ.x));

    if ((HalfA.x + GVector3::DotProduct( VAxis_Abs0, HalfB) ) < GMath::Abs(VDisBA.x))
        return false;

    GVector3 VAxis_Abs1 = GVector3(GMath::Abs(BLAX.y), GMath::Abs(BLAY.y), GMath::Abs(BLAZ.y));

    if ((HalfA.y + GVector3::DotProduct( VAxis_Abs1, HalfB) ) < GMath::Abs(VDisBA.y))
        return false;

    GVector3 VAxis_Abs2 = GVector3(GMath::Abs(BLAX.z), GMath::Abs(BLAY.z), GMath::Abs(BLAZ.z));

    if ((HalfA.z + GVector3::DotProduct( VAxis_Abs2, HalfB) ) < GMath::Abs(VDisBA.z))
        return false;

    const GVector3& VDisAB = B_to_A.m_Rotate.UnRotateVector( -B_to_A.m_Translation );

    const GVector3 VA_LocalB_X = GVector3(VAxis_Abs0.x, VAxis_Abs1.x, VAxis_Abs2.x );
    if ((HalfB.x + GVector3::DotProduct( VA_LocalB_X, HalfA)) < GMath::Abs(VDisAB.x))
        return false;

    const GVector3 VA_LocalB_Y = GVector3(VAxis_Abs0.y, VAxis_Abs1.y, VAxis_Abs2.y);
    if ((HalfB.y + GVector3::DotProduct( VA_LocalB_Y, HalfA)) < GMath::Abs(VDisAB.y))
        return false;

    const GVector3 VA_LocalB_Z = GVector3(VAxis_Abs0.z, VAxis_Abs1.z, VAxis_Abs2.z);
    if ((HalfB.z + GVector3::DotProduct( VA_LocalB_Z, HalfA)) < GMath::Abs(VDisAB.z))
        return false;

    BLAX *= HalfB.x;
    BLAY *= HalfB.y;
    BLAZ *= HalfB.z;

    GVector3 absBLAX = VA_LocalB_X * HalfB.x;
    GVector3 absBLAY = VA_LocalB_Y * HalfB.y;
    GVector3 absBLAZ = VA_LocalB_Z * HalfB.z;

    GVector3 absALBX = VAxis_Abs0 * HalfA.x;
    GVector3 absALBY = VAxis_Abs1 * HalfA.y;
    GVector3 absALBZ = VAxis_Abs2 * HalfA.z;

    if ((Dot_No_X(Cross_AbsX(HalfA.x, absBLAX), HalfA) + Dot_No_X(Cross_AbsX(HalfB.x, absALBX), HalfB)) < GMath::Abs(Dot_No_X(Cross_X(HalfA.x, BLAX), VDisBA)))  return false;
    if ((Dot_No_X(Cross_AbsX(HalfA.x, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBX), HalfB)) < GMath::Abs(Dot_No_X(Cross_X(HalfA.x, BLAY), VDisBA)))  return false;
    if ((Dot_No_X(Cross_AbsX(HalfA.x, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBX), HalfB)) < GMath::Abs(Dot_No_X(Cross_X(HalfA.x, BLAZ), VDisBA)))  return false;

    if ((Dot_No_Y(Cross_AbsY(HalfA.y, absBLAX), HalfA) + Dot_No_X(Cross_AbsX(HalfB.x, absALBY), HalfB)) < GMath::Abs(Dot_No_Y(Cross_Y(HalfA.y, BLAX), VDisBA)))  return false;
    if ((Dot_No_Y(Cross_AbsY(HalfA.y, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBY), HalfB)) < GMath::Abs(Dot_No_Y(Cross_Y(HalfA.y, BLAY), VDisBA)))  return false;
    if ((Dot_No_Y(Cross_AbsY(HalfA.y, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBY), HalfB)) < GMath::Abs(Dot_No_Y(Cross_Y(HalfA.y, BLAZ), VDisBA)))  return false;

    if ((Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAX), HalfA) + Dot_No_X(Cross_AbsX(HalfB.x, absALBZ), HalfB)) < GMath::Abs(Dot_No_Z(Cross_Z(HalfA.z, BLAX), VDisBA)))  return false;
    if ((Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBZ), HalfB)) < GMath::Abs(Dot_No_Z(Cross_Z(HalfA.z, BLAY), VDisBA)))  return false;
    if ((Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBZ), HalfB)) < GMath::Abs(Dot_No_Z(Cross_Z(HalfA.z, BLAZ), VDisBA)))  return false;

    return true;
}