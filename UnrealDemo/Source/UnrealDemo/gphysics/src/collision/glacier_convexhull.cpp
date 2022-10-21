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
#include "glacier_distance.h"

void GConvexHull::BuildEdges()
{
    m_Edges.clear();

    for (int i = 0; i < m_Polygons.size(); ++i)
    {
        const GConvexPolygon& polygenA = m_Polygons[i];
        for (int j = 0; j < i; ++j)
        {
            const GConvexPolygon& polygenB = m_Polygons[j];
            for (int nPA = 0; nPA < polygenA.m_NbVerts; ++nPA)
            {
                uint16_t IdA1 = m_Indexes[polygenA.m_IndexBase + nPA];
                uint16_t IdA2 = m_Indexes[polygenA.GetNext(nPA)];

                for (int nPB = 0; nPB < polygenB.m_NbVerts; ++nPB)
                {
                    uint16_t IdB1 = m_Indexes[polygenB.m_IndexBase + nPB];
                    uint16_t IdB2 = m_Indexes[polygenB.GetNext(nPB)];

                    if ((IdA1 == IdB1 && IdA2 == IdB2) || (IdA1 == IdB2 && IdA2 == IdB1))
                    {
                        GConvexEdge TEdge;
                        TEdge.PolygenId_A = i;
                        TEdge.PolygenId_B = j;

                        TEdge.PolygenA_Point_Index = nPA;
                        TEdge.PolygenB_Point_Index = nPB;

                        m_Edges.push_back(TEdge);
                    }
                }
            }
        }
    }

}


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




void GConvexHull::BuildMinkowskiSum(
    const GConvexHull&  CA,
    GTransform_QT       TA,
    const GConvexHull&  CB,
    GTransform_QT       TB,
    GConvexHull&        CResult )
{

    f32 fRel = GMath::Makef32(0,1,100);

    std::vector<GVector3> NewPoints;//.Clear();
   // SplitPoints.Clear();
   //NewIndecs.Clear();


    int32_t nAPointCount = (int32_t)CA.m_VPoints.size();
    int32_t nBPointCount = (int32_t)CB.m_VPoints.size();

    for (int i = 0; i < nAPointCount; ++i)
    {
        GVector3 Vi = TA.TransformPosition(CA.m_VPoints[i]);
        for (int j = 0; j < nBPointCount; ++j)
        {
            GVector3 Vj = TB.TransformPosition(CB.m_VPoints[i]);

            GVector3 VNew = Vi + Vj;

            bool bFind = false;
            for (int32_t n = 0; n < (int32_t)NewPoints.size(); n++)
            {
                f32 TVMax = (NewPoints[n] - VNew).AbsMax();

                if (TVMax < fRel )
                {
                    bFind = true;
                    break;
                }
            }
            if (!bFind)
                NewPoints.push_back(VNew);
        }
    }

   // CResult


    //Convexnew.BuildConvex(NewPoints, CResult.m_VPoints, NewIndecs);

}



void GConvexHullBuilder::BuildConvex(const std::vector<GVector3>& InputPoints, GConvexHull& CResult )
{
    m_VPoints.clear();
    m_Polygons.clear();
    CResult.Clear();

    AddInputPoints( InputPoints );

    f32 TEpsilon_Positive = GMath::Makef32(0, 1, 100000);
    f32 TEpsilon_Negative = -TEpsilon_Positive;

    for (int i = 0; i < (int32_t)m_VPoints.size(); i++)
    {
        const GVector3& Vi = m_VPoints[i];
        for (int j = 0; j < i; j++)
        {
            const GVector3& Vj = m_VPoints[j];
            GVector3 Vji = Vj - Vi;

            for (int k = 0; k < j; k++)
            {
                const GVector3& Vk = m_VPoints[k];
                GVector3 Vki = Vk - Vi;

                GVector3 Dir = GVector3::CrossProduct(Vji, Vki);

                f32 Distance = GVector3::DotProduct(Dir, Vi);

                int OP = 0;
                int Re = 0;

                for (int n = 0; n < (int32_t)m_VPoints.size(); n++)
                {
                    if (n == i || n == j || n == k)
                        continue;

                    f32 TDis = GVector3::DotProduct(Dir, m_VPoints[n]) - Distance;

                    if (TDis > TEpsilon_Positive) OP++;
                    if (TDis < TEpsilon_Negative) Re++;

                    if (OP > 0 && Re > 0)
                        break;
                }

                if ( (OP > 0 && Re == 0) || (OP == 0 && Re > 0) )
                {
                    GVector3 VNormal = Dir.GetNormalize();

                    if(Re == 0)
                        VNormal = -VNormal;

                    AddFace(GSortPlane( GPlane( VNormal, Vi ) ), i, j, k  );
                }
            }
        }
    }

    for (std::map<GSortPlane, GBuildPolygon>::iterator iter = m_Polygons.begin(); iter != m_Polygons.end(); ++iter)
    {
        SimpleliseFace(iter->second );
    }

}


void GConvexHullBuilder::Draw(class IGlacierDraw* pDraw, const GTransform_QT& Trans, GColor TColor) const
{
    for (int i = 0; i < m_VPoints.size(); ++i)
    {
        pDraw->DrawSphere( Trans.TransformPosition(m_VPoints[i] ), GMath::Inv_100(), TColor, 8 );
    }

    for (std::map<GSortPlane, GBuildPolygon>::const_iterator iter = m_Polygons.begin(); iter != m_Polygons.end(); ++iter)
    {
        for( int i = 0; i < iter->second.m_ListPoints.size(); ++i )
        {

            int iLast = i == 0 ? iter->second.m_ListPoints.size() - 1 : i - 1;

            int nIndexi = iter->second.m_ListPoints[i];
            int nIndexj = iter->second.m_ListPoints[iLast];


            pDraw->DrawLine( Trans.TransformPosition(m_VPoints[nIndexi] ), Trans.TransformPosition(m_VPoints[nIndexj]), TColor);
        }

        for (std::set<int32_t> ::iterator iterPos = iter->second.m_Points.begin(); iterPos != iter->second.m_Points.end(); ++iterPos)
        {
            pDraw->DrawLine( Trans.TransformPosition(iter->second.m_VCenter), Trans.TransformPosition(m_VPoints[*iterPos]), GColor::White());
        }

        pDraw->DrawArrow( Trans.TransformPosition(iter->second.m_VCenter), Trans.TransformNormal(iter->second.m_Plane.m_Normal), GMath::Inv_10(), TColor);
    }

}

void GConvexHullBuilder::AddInputPoints( const std::vector<GVector3>& InputPoints)
{
    int nInputCount = InputPoints.size();
    for (int i = 0; i < nInputCount; i++) // remove nearly points
    {
        const GVector3& TV = InputPoints[i];

        bool bFindnearly = false;
        for (int j = 0; j < m_VPoints.size(); ++j)
        {
            if ((TV - m_VPoints[j]).AbsMax() < GMath::Inv_1000())
            {
                bFindnearly = true;
                break;
            }
        }

        if( bFindnearly )
            continue;

        bool bisInline = false;

        for (int m = 0; m < m_VPoints.size(); ++m ) // remove three point collinear
        {
            if( m == i )
                continue;

            for (int n = 0; n < m; ++n)
            {
                 if( n == i ) 
                     continue;

                 GVector3 P1 = m_VPoints[m];
                 GVector3 P2 = m_VPoints[n];

                 if( GVector3::DistanceSquare(P1, P2) > GMath::Inv_100000() )
                 {
                     if (GVector3::DistanceSquare(GDistance::ClosestPtPointSegment(TV, P1, P2), TV) < GMath::Inv_100000())
                     {
                         bisInline = true;
                         break;
                     }
                 }

                 if( bisInline )
                 {
                    break;
                 }
            }
        }

        if( bisInline )
            continue;

        m_VPoints.push_back(TV);
    }
}

void GConvexHullBuilder::AddFace(const GSortPlane& TPlane, uint16_t i, uint16_t j, uint16_t k )
{
    GBuildPolygon* pPolygon = nullptr;

    std::map<GSortPlane, GBuildPolygon>::iterator Titer = m_Polygons.find( TPlane );
    if( Titer != m_Polygons.end() )
    {
        pPolygon = &(Titer->second);
    }
    else
    {
        pPolygon = &(m_Polygons[TPlane]);

        pPolygon->m_Plane = TPlane.m_Plane;
    }

    pPolygon->m_Points.insert( i );
    pPolygon->m_Points.insert( j );
    pPolygon->m_Points.insert( k );
}

void GConvexHullBuilder::SimpleliseFace(GBuildPolygon& TPolygon)
{
    m_PolygonPoints.clear();

    GVector3 VCenter = GVector3::Zero();
    for (std::set<int32_t> ::iterator iter = TPolygon.m_Points.begin(); iter != TPolygon.m_Points.end(); ++iter)
    {
        m_PolygonPoints.push_back( *iter );

        VCenter += m_VPoints[*iter ];
    }

    if(  m_PolygonPoints.size() != 0 )
    {
        VCenter /= f32(m_PolygonPoints.size());

        TPolygon.m_VCenter = VCenter;

        int32_t nStartPoint =  -1;

        f32 Tmax = GMath::Zero();

        for( int i = 0; i < m_PolygonPoints.size(); ++i )
        {
            f32 FDS = GVector3::DistanceSquare( m_VPoints[m_PolygonPoints[i]], VCenter );

            if( FDS > Tmax )
            {
                nStartPoint = m_PolygonPoints[i];
                Tmax = FDS;
            }       
        }

        if( nStartPoint != -1 )
        {
            TPolygon.m_ListPoints.clear();
            
            int32_t  nCurrent   =   nStartPoint;       
            GVector3 VDir       =   GVector3::CrossProduct( TPolygon.m_Plane.m_Normal, m_VPoints[nStartPoint] - VCenter );

            while( std::find(TPolygon.m_ListPoints.begin(), TPolygon.m_ListPoints.end(), nCurrent) == TPolygon.m_ListPoints.end())
            {
                TPolygon.m_ListPoints.push_back(nCurrent);

                GVector3 VCurrent   =   m_VPoints[nStartPoint];
                GVector3 VRight     =   GVector3::CrossProduct( TPolygon.m_Plane.m_Normal, VDir );

                f32 fAtanYX = -f32(100000);
                int32_t nFind = -1;
                for (int i = 0; i < m_PolygonPoints.size(); ++i)
                {
                    int32_t nTest = m_PolygonPoints[i];
                    if (nTest != nCurrent)
                    {
                        GVector3 VTest = m_VPoints[nTest];

                        GVector3 VDelta = VTest - VCurrent;

                        f32 fY = GVector3::DotProduct( VDir, VDelta);
                        f32 fX = GVector3::DotProduct( VRight, VDelta);

                        if( fX > GMath::Epsilon() )
                        {
                            f32 TYX = fY / fX;

                            if( TYX > fAtanYX )
                            {
                                fAtanYX = TYX;
                                nFind = nTest;
                            }
                        }
                        else
                        {
                            //todo
                        }
                    }
                }

                if( nFind != -1 )
                {
                    nCurrent = nFind;

                    VDir = m_VPoints[nFind] - VCurrent;
                }
                else
                {
                    break;
                }
            
            }
        }
    }
}
