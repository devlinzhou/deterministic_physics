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
#pragma once

#include "glacier_transform_qt.h"
#include "glacier_collision_shape.h"

class GCollision_Box
{
public:


    static bool Box_Box(
        const GShapeBox&        ShapA,
        const GTransform_QT&    TransformA,
        const GShapeBox&        ShapB,
        const GTransform_QT&    TransformB,
        GVector3*               pOutPosition,
        GVector3*               pOutNormal);

    static bool Box_Plane(
        const GShapeBox&        ShapA,
        const GTransform_QT&    TransformA,
        const GShapePlane&      ShapB,
        const GTransform_QT&    TransformB);
   

    static int32_t Box_Box_Contact(
        const GShapeBox&            ShapA,
        const GTransform_QT&        TransformA,
        const GShapeBox&            ShapB,
        const GTransform_QT&        TransformB,
        class GCollisionContact*    pContact);

    static int32_t Box_Box_Contact_PX(
        const GShapeBox& ShapA,
        const GTransform_QT& TransformA,
        const GShapeBox& ShapB,
        const GTransform_QT& TransformB,
        class GCollisionContact* pContact,
        bool& swap);


    static int32_t Plane_Box_Contact(
        const GShapePlane&          ShapA,
        const GTransform_QT&        TransformA,
        const GShapeBox&            ShapB,
        const GTransform_QT&        TransformB,
        class GCollisionContact*    pContact);
};


