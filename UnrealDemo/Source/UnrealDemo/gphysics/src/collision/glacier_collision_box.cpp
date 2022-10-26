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
    f32 absDot1 = GMath::Abs(GVector3::DotProduct(VAxis, V1X)) + GMath::Abs(GVector3::DotProduct(VAxis, V1Y)) + GMath::Abs(GMath::Abs(GVector3::DotProduct(VAxis, V1Z)));
    f32 absDot2 = GMath::Abs(GVector3::DotProduct(VAxis, V2X)) + GMath::Abs(GVector3::DotProduct(VAxis, V2Y)) + GMath::Abs(GMath::Abs(GVector3::DotProduct(VAxis, V2Z)));
    f32 absDot3 = GMath::Abs(GVector3::DotProduct(VAxis, VDis));

    return (absDot1 + absDot2 ) < absDot3;
}

bool GCollision_Box::Box_Box(
    const GShapeBox& ShapA, const GTransform_QT& TransformA,
    const GShapeBox& ShapB, const GTransform_QT& TransformB,
    GVector3* pOutPosition, GVector3* pOutNormal)
{
    GTransform_QT   B_to_A      = TransformB * TransformA.GetInverse_fast();
    const GVector3& VA_LocalA   = ShapA.HalfExtern;

    GMatrix3        M_B_To_A( B_to_A.m_Rotate );

    GVector3        VBLAX = M_B_To_A.GetRow(0) * ShapB.HalfExtern.x;
    GVector3        VBLAY = M_B_To_A.GetRow(1) * ShapB.HalfExtern.y;
    GVector3        VBLAZ = M_B_To_A.GetRow(2) * ShapB.HalfExtern.z;

    if ((VA_LocalA.x + f32::Abs(VBLAX.x) + f32::Abs(VBLAY.x) + f32::Abs(VBLAZ.x)) < f32::Abs(B_to_A.m_Translation.x))
        return false;

    if ((VA_LocalA.y + f32::Abs(VBLAX.y) + f32::Abs(VBLAY.y) + f32::Abs(VBLAZ.y)) < f32::Abs(B_to_A.m_Translation.y))
        return false;

    if ((VA_LocalA.z + f32::Abs(VBLAX.z) + f32::Abs(VBLAY.z) + f32::Abs(VBLAZ.z))< f32::Abs(B_to_A.m_Translation.z))
        return false;

    GTransform_QT   A_to_B = TransformA * TransformB.GetInverse_fast();
    const GVector3& VB_LocalB = ShapB.HalfExtern;
    GMatrix3        M_A_To_B( A_to_B.m_Rotate );

    GVector3        VA_LocalB_X = M_A_To_B.GetRow(0) * ShapA.HalfExtern.x;
    GVector3        VA_LocalB_Y = M_A_To_B.GetRow(1) * ShapA.HalfExtern.y;
    GVector3        VA_LocalB_Z = M_A_To_B.GetRow(2) * ShapA.HalfExtern.z;
    
    if ((VB_LocalB.x + f32::Abs(VA_LocalB_X.x) + f32::Abs(VA_LocalB_Y.x) + f32::Abs(VA_LocalB_Z.x)) < f32::Abs(A_to_B.m_Translation.x))
        return false;

    if ((VB_LocalB.y + f32::Abs(VA_LocalB_X.y) + f32::Abs(VA_LocalB_Y.y) + f32::Abs(VA_LocalB_Z.y))< f32::Abs(A_to_B.m_Translation.y))
        return false;

    if ((VB_LocalB.z + f32::Abs(VA_LocalB_X.z) + f32::Abs(VA_LocalB_Y.z) + f32::Abs(VA_LocalB_Z.z)) < f32::Abs(A_to_B.m_Translation.z))
        return false;

    GVector3 VALAX = GVector3(VA_LocalA.x, GMath::Zero(), GMath::Zero());
    GVector3 VALAY = GVector3(GMath::Zero(), VA_LocalA.y, GMath::Zero());
    GVector3 VALAZ = GVector3(GMath::Zero(), GMath::Zero(), VA_LocalA.z);

    if (ProjectAxistest(GVector3::CrossProduct(VALAX, VBLAX), VALAX, VALAY, VALAZ, VBLAX, VBLAY, VBLAZ, B_to_A.m_Translation)) return false;
    if (ProjectAxistest(GVector3::CrossProduct(VALAX, VBLAY), VALAX, VALAY, VALAZ, VBLAX, VBLAY, VBLAZ, B_to_A.m_Translation)) return false;
    if (ProjectAxistest(GVector3::CrossProduct(VALAX, VBLAZ), VALAX, VALAY, VALAZ, VBLAX, VBLAY, VBLAZ, B_to_A.m_Translation)) return false;
    if (ProjectAxistest(GVector3::CrossProduct(VALAY, VBLAX), VALAX, VALAY, VALAZ, VBLAX, VBLAY, VBLAZ, B_to_A.m_Translation)) return false;
    if (ProjectAxistest(GVector3::CrossProduct(VALAY, VBLAY), VALAX, VALAY, VALAZ, VBLAX, VBLAY, VBLAZ, B_to_A.m_Translation)) return false;
    if (ProjectAxistest(GVector3::CrossProduct(VALAY, VBLAZ), VALAX, VALAY, VALAZ, VBLAX, VBLAY, VBLAZ, B_to_A.m_Translation)) return false;
    if (ProjectAxistest(GVector3::CrossProduct(VALAZ, VBLAX), VALAX, VALAY, VALAZ, VBLAX, VBLAY, VBLAZ, B_to_A.m_Translation)) return false;
    if (ProjectAxistest(GVector3::CrossProduct(VALAZ, VBLAY), VALAX, VALAY, VALAZ, VBLAX, VBLAY, VBLAZ, B_to_A.m_Translation)) return false;
    if (ProjectAxistest(GVector3::CrossProduct(VALAZ, VBLAZ), VALAX, VALAY, VALAZ, VBLAX, VBLAY, VBLAZ, B_to_A.m_Translation)) return false;




    return true;

   /* if ((VA_LocalA.x + f32::Abs(VB_LocalA.x)) < f32::Abs(B_to_A.m_Translation.x))
        return false;

    if ((VA_LocalA.y + f32::Abs(VB_LocalA.y)) < f32::Abs(B_to_A.m_Translation.y))
        return false;

    if ((VA_LocalA.z + f32::Abs(VB_LocalA.z)) < f32::Abs(B_to_A.m_Translation.z))
        return false;

    GTransform_QT   A_to_B      = TransformA * TransformB.GetInverse_fast();
    const GVector3& VB_LocalB   = ShapB.HalfExtern;
    GVector3        VA_LocalB   = A_to_B.TransformNormal(ShapA.HalfExtern);

    if ((VB_LocalB.x + f32::Abs(VA_LocalB.x)) < f32::Abs(A_to_B.m_Translation.x))
        return false;

    if ((VB_LocalB.y + f32::Abs(VA_LocalB.y)) < f32::Abs(A_to_B.m_Translation.y))
        return false;

    if ((VB_LocalB.z + f32::Abs(VA_LocalB.z)) < f32::Abs(A_to_B.m_Translation.z))
        return false;


    return true;*/
}