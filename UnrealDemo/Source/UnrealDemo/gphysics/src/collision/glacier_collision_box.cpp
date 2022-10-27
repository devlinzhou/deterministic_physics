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


inline bool ProjectAxistest( const GVector3& VAxis,
    const GVector3& V1X, const GVector3& V1Y, const GVector3& V1Z,
    const GVector3& V2X, const GVector3& V2Y, const GVector3& V2Z,
    const GVector3& VDis )
{
    f32 absDot1 = GMath::Abs(GVector3::DotProduct(VAxis, V1X)) + GMath::Abs(GVector3::DotProduct(VAxis, V1Y)) + GMath::Abs((GVector3::DotProduct(VAxis, V1Z)));
    f32 absDot2 = GMath::Abs(GVector3::DotProduct(VAxis, V2X)) + GMath::Abs(GVector3::DotProduct(VAxis, V2Y)) + GMath::Abs((GVector3::DotProduct(VAxis, V2Z)));
    f32 absDot3 = GMath::Abs(GVector3::DotProduct(VAxis, VDis));

    return (absDot1 + absDot2 ) < absDot3;
}

static inline GVector3 Cross_X(f32 a, const GVector3& V)
{
    return GVector3(GMath::Zero(), -a * V.z, a * V.y);
}

static inline GVector3 Cross_Y(f32 a, const GVector3& V)
{
    return  GVector3(a * V.z , GMath::Zero(), - a * V.x);
}

static inline GVector3 Cross_Z(f32 a, const GVector3& V)
{
    return GVector3(- a * V.y, a * V.x, GMath::Zero());
}

static inline f32 DotProduct_YZ(const GVector3& V1, const GVector3& V2)
{
    return V1.y * V2.y + V1.z * V2.z;
}

static inline f32 DotProduct_ZX(const GVector3& V1, const GVector3& V2)
{
    return V1.x * V2.x + V1.z * V2.z;
}

static inline f32 DotProduct_XY(const GVector3& V1, const GVector3& V2)
{
    return V1.x * V2.x + V1.y * V2.y;
}

inline bool ProjectAxisTest_X_Axis(
    const GVector3& VAxis,
    const GVector3& V1,
    const GVector3& V2A, 
    const GVector3& V2B,
    const GVector3& VDis)
{
    f32 absDot1 = V1.y * GMath::Abs( VAxis.y ) + V1.z * GMath::Abs(VAxis.z );

    f32 absDot2 = GMath::Abs(DotProduct_YZ(VAxis, V2A )) + GMath::Abs(DotProduct_YZ(VAxis, V2B));
    
    f32 absDot3 = GMath::Abs(DotProduct_YZ(VAxis, VDis));

    return (absDot1 + absDot2) < absDot3;
}

inline f32 Dot_No_X( const GVector3& SatAxisLocal, const GVector3& HalfExten )
{
    return HalfExten.y * GMath::Abs(SatAxisLocal.y) + HalfExten.z * GMath::Abs(SatAxisLocal.z);
}

inline f32 Dot_No_Y(const GVector3& SatAxisLocal, const GVector3& HalfExten)
{
    return HalfExten.x * GMath::Abs(SatAxisLocal.x) + HalfExten.z * GMath::Abs(SatAxisLocal.z);
}

inline f32 Dot_No_Z(const GVector3& SatAxisLocal, const GVector3& HalfExten)
{
    return HalfExten.x * GMath::Abs(SatAxisLocal.x) + HalfExten.y * GMath::Abs(SatAxisLocal.y);
}


inline bool ProjectTest_X(
    const GVector3& SatAxisLocalA,
    const GVector3& HalfExtenA,
    f32             absDot2,
    const GVector3& LocalADis)
{
    f32 absDot1 = Dot_No_X(SatAxisLocalA, HalfExtenA);
    f32 absDot3 = GMath::Abs(DotProduct_YZ(SatAxisLocalA, LocalADis));
    return (absDot1 + absDot2) < absDot3;
}

inline bool ProjectTest_Y(
    const GVector3& SatAxisLocalA,
    const GVector3& HalfExtenA,
    f32             absDot2,
    const GVector3& LocalADis)
{
    f32 absDot1 = Dot_No_Y(SatAxisLocalA, HalfExtenA);
    f32 absDot3 = GMath::Abs(DotProduct_ZX(SatAxisLocalA, LocalADis));
    return (absDot1 + absDot2) < absDot3;
}

inline bool ProjectTest_Z(
    const GVector3& SatAxisLocalA,
    const GVector3& HalfExtenA,
    f32             absDot2,
    const GVector3& LocalADis)
{
    f32 absDot1 = Dot_No_Z(SatAxisLocalA, HalfExtenA);
    f32 absDot3 = GMath::Abs(DotProduct_XY(SatAxisLocalA, LocalADis));
    return (absDot1 + absDot2) < absDot3;
}


inline bool ProjectAxisTest_Y_Axis(const GVector3& VAxis,
    const GVector3& V1,
    const GVector3& V2A, const GVector3& V2B,
    const GVector3& VDis)
{
    f32 absDot1 = V1.x * GMath::Abs( VAxis.x) + V1.z * GMath::Abs( VAxis.z);

    f32 absDot2 =  GMath::Abs(DotProduct_ZX(VAxis, V2A  ))  + GMath::Abs(DotProduct_ZX(VAxis, V2B));

    f32 absDot3 = GMath::Abs(DotProduct_ZX(VAxis, VDis));

    return (absDot1 + absDot2) < absDot3;
}

inline bool ProjectAxisTest_Z_Axis(const GVector3& VAxis,
    const GVector3& V1,
    const GVector3& V2A, const GVector3& V2B,
    const GVector3& VDis)
{
    f32 absDot1 = V1.x * GMath::Abs( VAxis.x) + V1.y * GMath::Abs( VAxis.y);

    f32 absDot2 =  GMath::Abs(DotProduct_XY(VAxis, V2A ))  + GMath::Abs(DotProduct_XY(VAxis, V2B));

    f32 absDot3 = GMath::Abs(DotProduct_XY(VAxis, VDis));

    return (absDot1 + absDot2) < absDot3;
}

bool GCollision_Box::Box_Box(
    const GShapeBox& ShapA, const GTransform_QT& TransformA,
    const GShapeBox& ShapB, const GTransform_QT& TransformB,
    GVector3* pOutPosition, GVector3* pOutNormal)
{
    GTransform_QT   B_to_A      = TransformB * TransformA.GetInverse_fast();

    GMatrix3        M_B_To_A( B_to_A.m_Rotate );

    GVector3 VBLAX      = M_B_To_A.GetRow(0);
    GVector3 VBLAY      = M_B_To_A.GetRow(1);
    GVector3 VBLAZ      = M_B_To_A.GetRow(2);
    GVector3& VDisBA    = B_to_A.m_Translation;

    const GVector3& ExternA   = ShapA.HalfExtern;
    const GVector3& ExternB   = ShapB.HalfExtern;

    GVector3 VAxis_Abs0 = GVector3(GMath::Abs(VBLAX.x), GMath::Abs(VBLAY.x), GMath::Abs(VBLAZ.x));

    if ((ExternA.x + GVector3::DotProduct( VAxis_Abs0, ExternB) ) < GMath::Abs(VDisBA.x))
        return false;

    GVector3 VAxis_Abs1 = GVector3(GMath::Abs(VBLAX.y), GMath::Abs(VBLAY.y), GMath::Abs(VBLAZ.y));

    if ((ExternA.y + GVector3::DotProduct( VAxis_Abs1, ExternB) ) < GMath::Abs(VDisBA.y))
        return false;

    GVector3 VAxis_Abs2 = GVector3(GMath::Abs(VBLAX.z), GMath::Abs(VBLAY.z), GMath::Abs(VBLAZ.z));

    if ((ExternA.z + GVector3::DotProduct( VAxis_Abs2, ExternB) ) < GMath::Abs(VDisBA.z))
        return false;

    const GVector3& VDisAB = B_to_A.m_Rotate.UnRotateVector( -B_to_A.m_Translation );

    const GVector3 VA_LocalB_X = GVector3(VAxis_Abs0.x, VAxis_Abs1.x, VAxis_Abs2.x );
    if ((ExternB.x + GVector3::DotProduct( VA_LocalB_X, ExternA)) < GMath::Abs(VDisAB.x))
        return false;

    const GVector3 VA_LocalB_Y = GVector3(VAxis_Abs0.y, VAxis_Abs1.y, VAxis_Abs2.y);
    if ((ExternB.y + GVector3::DotProduct( VA_LocalB_Y, ExternA)) < GMath::Abs(VDisAB.y))
        return false;

    const GVector3 VA_LocalB_Z = GVector3(VAxis_Abs0.z, VAxis_Abs1.z, VAxis_Abs2.z);
    if ((ExternB.z + GVector3::DotProduct( VA_LocalB_Z, ExternA)) < GMath::Abs(VDisAB.z))
        return false;

     VBLAX = VBLAX * ShapB.HalfExtern.x;
     VBLAY = VBLAY * ShapB.HalfExtern.y;
     VBLAZ = VBLAZ * ShapB.HalfExtern.z;

     GVector3 VALBX = M_B_To_A.GetColumn(0);
     GVector3 VALBY = M_B_To_A.GetColumn(1);
     GVector3 VALBZ = M_B_To_A.GetColumn(2);


    VALBX = VALBX * ShapA.HalfExtern.x;
    VALBY = VALBY * ShapA.HalfExtern.y;
    VALBZ = VALBZ * ShapA.HalfExtern.z;

    if (ProjectTest_X(Cross_X(ExternA.x, VBLAX), ExternA, Dot_No_X(Cross_X(ExternB.x, VALBX), ExternB), VDisBA)) return false;
    if (ProjectTest_X(Cross_X(ExternA.x, VBLAY), ExternA, Dot_No_Y(Cross_Y(ExternB.y, VALBX), ExternB), VDisBA)) return false;
    if (ProjectTest_X(Cross_X(ExternA.x, VBLAZ), ExternA, Dot_No_Z(Cross_Z(ExternB.z, VALBX), ExternB), VDisBA)) return false;

    if (ProjectTest_Y(Cross_Y(ExternA.y, VBLAX), ExternA, Dot_No_X(Cross_X(ExternB.x, VALBY), ExternB), VDisBA)) return false;
    if (ProjectTest_Y(Cross_Y(ExternA.y, VBLAY), ExternA, Dot_No_Y(Cross_Y(ExternB.y, VALBY), ExternB), VDisBA)) return false;
    if (ProjectTest_Y(Cross_Y(ExternA.y, VBLAZ), ExternA, Dot_No_Z(Cross_Z(ExternB.z, VALBY), ExternB), VDisBA)) return false;

    if (ProjectTest_Z(Cross_Z(ExternA.y, VBLAX), ExternA, Dot_No_X(Cross_X(ExternB.x, VALBZ), ExternB), VDisBA)) return false;
    if (ProjectTest_Z(Cross_Z(ExternA.y, VBLAY), ExternA, Dot_No_Y(Cross_Y(ExternB.y, VALBZ), ExternB), VDisBA)) return false;
    if (ProjectTest_Z(Cross_Z(ExternA.y, VBLAZ), ExternA, Dot_No_Z(Cross_Z(ExternB.z, VALBZ), ExternB), VDisBA)) return false;

//     if (ProjectAxisTest_X_Axis(Cross_AxisX(VA_LocalA.x, VBLAX), VA_LocalA, VBLAY, VBLAZ, VDisBA)) return false;
//     if (ProjectAxisTest_X_Axis(Cross_AxisX(VA_LocalA.x, VBLAY), VA_LocalA, VBLAX, VBLAZ, VDisBA)) return false;
//     if (ProjectAxisTest_X_Axis(Cross_AxisX(VA_LocalA.x, VBLAZ), VA_LocalA, VBLAX, VBLAY, VDisBA)) return false;
//     if (ProjectAxisTest_Y_Axis(Cross_AxisY(VA_LocalA.y, VBLAX), VA_LocalA, VBLAY, VBLAZ, VDisBA)) return false;
//     if (ProjectAxisTest_Y_Axis(Cross_AxisY(VA_LocalA.y, VBLAY), VA_LocalA, VBLAX, VBLAZ, VDisBA)) return false;
//     if (ProjectAxisTest_Y_Axis(Cross_AxisY(VA_LocalA.y, VBLAZ), VA_LocalA, VBLAX, VBLAY, VDisBA)) return false;
//     if (ProjectAxisTest_Z_Axis(Cross_AxisZ(VA_LocalA.z, VBLAX), VA_LocalA, VBLAY, VBLAZ, VDisBA)) return false;
//     if (ProjectAxisTest_Z_Axis(Cross_AxisZ(VA_LocalA.z, VBLAY), VA_LocalA, VBLAX, VBLAZ, VDisBA)) return false;
//     if (ProjectAxisTest_Z_Axis(Cross_AxisZ(VA_LocalA.z, VBLAZ), VA_LocalA, VBLAX, VBLAY, VDisBA)) return false;


    return true;
}