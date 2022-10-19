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

#include "glacier_convexhull.h"
#include "glacier_debug_draw.h"


void GConvexHull::Draw(class IGlacierDraw* pDraw, const GTransform_QT& Trans, GColor TColor ) const
{
    if (m_Edges.size() != 0)
    {
        f32 bias = GMath::Makef32(0,8,10);

        for (int i = 0; i < m_Edges.size(); i++)
        {
            const GConvexEdge& TEdge = m_Edges[i];

            const GConvexPolygon& TPolygonA = m_Polygons[TEdge.PolygenId_A];
            const GConvexPolygon& TPolygonB = m_Polygons[TEdge.PolygenId_B];

            GVector3 VA1 = m_VPoints[m_Indexes[TPolygonA.m_IndexBase + TEdge.PolygenA_Point_Index]];
            GVector3 VA2 = m_VPoints[m_Indexes[TPolygonA.GetNext(TEdge.PolygenA_Point_Index)]];

            GVector3 VA1_Draw = VA1 + (TPolygonA.m_CenterPos - VA1).GetNormalize() * bias;
            GVector3 VA2_Draw = VA2 + (TPolygonA.m_CenterPos - VA2).GetNormalize() * bias;

            GVector3 VB1 = m_VPoints[m_Indexes[TPolygonB.m_IndexBase + TEdge.PolygenB_Point_Index]];
            GVector3 VB2 = m_VPoints[m_Indexes[TPolygonB.GetNext(TEdge.PolygenB_Point_Index)]];

            GVector3 VB1_Draw = VB1 + (TPolygonB.m_CenterPos - VB1).GetNormalize() * bias;
            GVector3 VB2_Draw = VB2 + (TPolygonB.m_CenterPos - VB2).GetNormalize() * bias;

            pDraw->DrawLine(VA1_Draw, VA2_Draw, TColor);
            pDraw->DrawLine(VB1_Draw, VB2_Draw, TColor);

        }
    }
    else
    {

        for (int i = 0; i < m_Polygons.size(); i++)
        {
            const GConvexPolygon& TPolygon = m_Polygons[i];

            GVector3 VLast = Trans.TransformPosition(m_VPoints[m_Indexes[TPolygon.m_IndexBase + TPolygon.m_NbVerts - 1]]);

            for (int j = 0; j < TPolygon.m_NbVerts; ++j)
            {
                GVector3 VCurrent = Trans.TransformPosition(m_VPoints[m_Indexes[TPolygon.m_IndexBase + j]]);
  
                pDraw->DrawLine(VLast, VCurrent, TColor);
                VLast = VCurrent;
            }

            GVector3 VCenter = Trans.TransformPosition(TPolygon.m_CenterPos);

            pDraw->DrawArrow( VCenter, Trans.TransformNormal(TPolygon.m_Plane.m_Normal), GMath::Makef32(8,0,100), TColor);
        }
    }
}