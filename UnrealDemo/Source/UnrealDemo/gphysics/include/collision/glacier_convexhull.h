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
#include <set>
#include <map>


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
    virtual ~GConvexHull()
    {
    
    }

    void Clear()
    {
        m_VPoints.clear();
        m_Indexes.clear();
        m_Polygons.clear();
        m_Edges.clear();
    }

    virtual GVector3 GetSupportLocalPos(const GVector3& Dir) const
    {
        return Dir;// * Radius;
    }

    void BuildEdges();

    void Draw( class IGlacierDraw* pDraw, const GTransform_QT& Trans, GColor TColor ) const;

protected:

    std::vector<GVector3>       m_VPoints;
    std::vector<uint16_t>       m_Indexes;
    std::vector<GConvexPolygon> m_Polygons;
    std::vector<GConvexEdge>    m_Edges;


    friend class GConvexHullBuilder;
};


class GSortPlane
{
public:

    GPlane m_Plane;

    GSortPlane()
    {

    }

    GSortPlane(GPlane TPlane) : m_Plane(TPlane)
    {

    }

    GSortPlane(const GSortPlane&) = default;

    inline bool operator == (const GSortPlane& b) const
    {
        return  
            GVector3::DistanceSquare( m_Plane.m_Normal, b.m_Plane.m_Normal ) < GMath::Inv_1000000() &&
            GMath::Abs( m_Plane.m_fDis - b.m_Plane.m_fDis ) < GMath::Inv_1000();
    }

    inline bool operator != (const GSortPlane& b) const
    {
        return !( *this == b );
    }

    inline bool operator < (const GSortPlane& b) const
    {
        if( *this == b )
            return false;
        else
        {
            const GVector3& Va = m_Plane.m_Normal;
            const GVector3& Vb = b.m_Plane.m_Normal;

            if (Va.z != Vb.z)
            {
                return Va.z < Vb.z;
            }
            else if (Va.y != Vb.y)
            {
                return Va.y < Vb.y;
            }
            else if (Va.x != Vb.x)
            {
                return Va.x < Vb.x;
            }
            else
            {
                return m_Plane.m_fDis < b.m_Plane.m_fDis;
            }
        }
    }
};


struct GBuildPolygon
{
    GPlane                  m_Plane;
    GVector3                m_VCenter;
    std::set<int32_t>       m_Points;
    std::vector<int32_t>    m_ListPoints;
};

class GConvexHullBuilder
{
public:


    static void AddBoxPoints(std::vector<GVector3>& Points, const GVector3& VMin, const GVector3& VMax );

    void BuildConvex(const std::vector<GVector3>& Points, GConvexHull& CResult );

    void BuildMinkowskiSum(
        const GConvexHull&  CA,
        GTransform_QT       TA,
        const GConvexHull&  CB,
        GTransform_QT       TB,
        GConvexHull&        CResult,
        bool                bAdd = true);


    void Draw(class IGlacierDraw* pDraw, const GTransform_QT& Trans, GColor TColor) const;

protected:


    void AddInputPoints(const std::vector<GVector3>& InputPoints);

    void AddFace(const GSortPlane& TPlane, uint16_t i, uint16_t j, uint16_t k);



    void SimpleliseFace( GBuildPolygon& TPolygon );


public:
    std::vector<GVector3>                   m_VPoints;
    std::map<GSortPlane, GBuildPolygon>     m_Polygons;


    std::vector<int32_t>                   m_PolygonPoints;

};