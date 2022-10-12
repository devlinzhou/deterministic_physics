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

#include "glacier_contact.h"



void GCollisionContact::AddContactPoint(const GVector3& PosWorld, const GVector3& NormalOnB, f32 depth)
{
    if(0<= m_nPointCount && m_nPointCount < 4 )
    {
        GManifoldPoint& TPoint = m_Point[m_nPointCount-1];

        TPoint.m_PosWorld = PosWorld;
        TPoint.m_NormalOnB = NormalOnB;
        TPoint.m_depth = depth;
    }

}
