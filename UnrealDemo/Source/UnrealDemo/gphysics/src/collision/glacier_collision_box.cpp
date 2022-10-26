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

bool GCollision_Box::Box_Box(
    const GShapeBox& ShapA, const GTransform_QT& TransformA,
    const GShapeBox& ShapB, const GTransform_QT& TransformB,
    GVector3* pOutPosition, GVector3* pOutNormal)
{
    GTransform_QT   B_to_A      = TransformB * TransformA.GetInverse_fast();
    const GVector3& VA_LocalA   = ShapA.HalfExtern;
    GVector3        VB_LocalA   = B_to_A.TransformNormal( ShapB.HalfExtern);

    GVector3        VB_LocalA_X   = B_to_A.TransformNormal( GVector3( ShapB.HalfExtern.x,   GMath::Zero(),      GMath::Zero()));
    GVector3        VB_LocalA_Y   = B_to_A.TransformNormal( GVector3( GMath::Zero(),        ShapB.HalfExtern.y, GMath::Zero()));
    GVector3        VB_LocalA_Z   = B_to_A.TransformNormal( GVector3( GMath::Zero(),        GMath::Zero(),      ShapB.HalfExtern.z));

    if ((VA_LocalA.x + f32::Abs(VB_LocalA_X.x) + f32::Abs(VB_LocalA_Y.x) + f32::Abs(VB_LocalA_Z.x) ) < f32::Abs(B_to_A.m_Translation.x))
        return false;

    if ((VA_LocalA.y + f32::Abs(VB_LocalA_X.y)) + f32::Abs(VB_LocalA_Y.y) + f32::Abs(VB_LocalA_Z.y) < f32::Abs(B_to_A.m_Translation.y))
        return false;

    if ((VA_LocalA.z + f32::Abs(VB_LocalA_X.z)) + f32::Abs(VB_LocalA_Y.z) + f32::Abs(VB_LocalA_Z.z) < f32::Abs(B_to_A.m_Translation.z))
        return false;

    GTransform_QT   A_to_B = TransformA * TransformB.GetInverse_fast();
    const GVector3& VB_LocalB = ShapB.HalfExtern;
    GVector3        VA_LocalB = A_to_B.TransformNormal(ShapA.HalfExtern);

    GVector3        VA_LocalB_X = A_to_B.TransformNormal(GVector3(ShapA.HalfExtern.x, GMath::Zero(), GMath::Zero()));
    GVector3        VA_LocalB_Y = A_to_B.TransformNormal(GVector3(GMath::Zero(), ShapA.HalfExtern.y, GMath::Zero()));
    GVector3        VA_LocalB_Z = A_to_B.TransformNormal(GVector3(GMath::Zero(), GMath::Zero(), ShapA.HalfExtern.z));
    
    if ((VB_LocalB.x + f32::Abs(VA_LocalB_X.x) + f32::Abs(VA_LocalB_Y.x) + f32::Abs(VA_LocalB_Z.x) ) < f32::Abs(A_to_B.m_Translation.x))
        return false;

    if ((VB_LocalB.y + f32::Abs(VA_LocalB_X.y)) + f32::Abs(VA_LocalB_Y.y) + f32::Abs(VA_LocalB_Z.y) < f32::Abs(A_to_B.m_Translation.y))
        return false;

    if ((VB_LocalB.z + f32::Abs(VA_LocalB_X.z)) + f32::Abs(VA_LocalB_Y.z) + f32::Abs(VA_LocalB_Z.z) < f32::Abs(A_to_B.m_Translation.z))
        return false;




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