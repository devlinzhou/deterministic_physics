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
    GTransform_QT A_to_B = TransformA * TransformB.GetInverse();
    GTransform_QT B_to_A = TransformB * TransformA.GetInverse();


    const GVector3& VA_LocalA = ShapA.HalfExtern;
    GVector3        VB_LocalA = B_to_A.TransformNormal( ShapB.HalfExtern);




    return false;
}