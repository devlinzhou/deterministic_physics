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

#include "glacier_vector.h"
#include "glacier_transform_qt.h"
#include "glacier_color.h"
#include "glacier_matrix.h"
#include "glacier_math.h"
#include "glacier_collision_shape.h"

class GCollisionShape;
class IGlacierDraw;
class GCollisionContact;
class GPhysicsWorld;
class GPhyscsUtils
{
public:

     static void DrawShape( const GTransform_QT& Trans, const GCollisionShape& pShape, IGlacierDraw* pDebugDraw, GColor TColor );

     static void DrawSphere(const GTransform_QT& Trans, const GShapeSphere& pShape, IGlacierDraw* pDebugDraw, GColor TColor);

     static void DrawBox(const GTransform_QT& Trans, const GShapeBox& pShape, IGlacierDraw* pDebugDraw, GColor TColor);

     static void DrawCapsule(const GTransform_QT& Trans, const GShapeCapsule& pShape, IGlacierDraw* pDebugDraw, GColor TColor);
     
     static void DrawCoordinateSystem( IGlacierDraw* pDebugDraw, const GTransform_QT& Trans, f32 fSize );

     static void DrawContact( const GCollisionContact& TContact, IGlacierDraw* pDebugDraw, GColor TColor, const class GPhysicsWorld* = nullptr );

     static GMatrix3 CalculateInertiaTensor( const GCollisionShape& pShape );

     static f32 CalculateVolume(const GCollisionShape& pShape);



};