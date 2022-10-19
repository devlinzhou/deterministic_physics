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
#include "glacier_plane.h"
#include "glacier_transform_qt.h"
#include "glacier_collision_shape.h"
#include "glacier_debug_draw.h"


#include <vector>

struct GConvexPolygon
{
    GPlane          m_Plane;
    GVector3        m_CenterPos;
    uint16_t		m_NbVerts;
    uint16_t		m_IndexBase;

    inline uint16_t GetNext(uint16_t nCurrentindex) const
    {
        return m_IndexBase + (nCurrentindex < (m_NbVerts - 1) ? nCurrentindex + 1 : 0);
    }
};

struct GConvexEdge
{
    uint16_t PolygenId_A;
    uint16_t PolygenId_B;

    uint16_t PolygenA_Point_Index;
    uint16_t PolygenB_Point_Index;
};



class GConvexHull : public GShapeConvexBase
{
public:
    GConvexHull(  )
    {
        ShapType = EShape::EShape_ConvexHull;
    }

    virtual GVector3 GetSupportLocalPos(const GVector3& Dir) const
    {
        return Dir;// * Radius;
    }

    void BuildEdges();


    void Draw( class IGlacierDraw* pDraw, const GTransform_QT& Trans, GColor TColor ) const;

    std::vector<GVector3>       m_VPoints;
    std::vector<uint16_t>       m_Indexes;
    std::vector<GConvexPolygon> m_Polygons;
    std::vector<GConvexEdge>    m_Edges;

};