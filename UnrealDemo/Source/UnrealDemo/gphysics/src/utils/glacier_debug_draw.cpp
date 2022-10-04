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

#include "glacier_debug_draw.h"
#include <vector>

void IGLacierDraw::DrawBox(const GTransform_QT& TTrans, const GVector3& LocalCenter, const GVector3& HalfSize, uint32_t TColor)
{
    GVector3 c = LocalCenter;
    GVector3 d = HalfSize;
    GVector3 dx = GVector3(d.x, GMath::Zero(), GMath::Zero());
    GVector3 dy = GVector3(GMath::Zero(), d.y, GMath::Zero());
    GVector3 dz = GVector3(GMath::Zero(), GMath::Zero(), d.z);

    DrawLine(TTrans.TransformPosition(c + dy + dz - dx), TTrans.TransformPosition(c + dy + dz + dx), TColor);
    DrawLine(TTrans.TransformPosition(c + dy - dz - dx), TTrans.TransformPosition(c + dy - dz + dx), TColor);
    DrawLine(TTrans.TransformPosition(c - dy + dz - dx), TTrans.TransformPosition(c - dy + dz + dx), TColor);
    DrawLine(TTrans.TransformPosition(c - dy - dz - dx), TTrans.TransformPosition(c - dy - dz + dx), TColor);
    DrawLine(TTrans.TransformPosition(c + dy + dx - dz), TTrans.TransformPosition(c + dy + dx + dz), TColor);
    DrawLine(TTrans.TransformPosition(c + dy - dx - dz), TTrans.TransformPosition(c + dy - dx + dz), TColor);
    DrawLine(TTrans.TransformPosition(c - dy + dx - dz), TTrans.TransformPosition(c - dy + dx + dz), TColor);
    DrawLine(TTrans.TransformPosition(c - dy - dx - dz), TTrans.TransformPosition(c - dy - dx + dz), TColor);
    DrawLine(TTrans.TransformPosition(c + dz + dx - dy), TTrans.TransformPosition(c + dz + dx + dy), TColor);
    DrawLine(TTrans.TransformPosition(c + dz - dx - dy), TTrans.TransformPosition(c + dz - dx + dy), TColor);
    DrawLine(TTrans.TransformPosition(c - dz + dx - dy), TTrans.TransformPosition(c - dz + dx + dy), TColor);
    DrawLine(TTrans.TransformPosition(c - dz - dx - dy), TTrans.TransformPosition(c - dz - dx + dy), TColor);
}

void IGLacierDraw::DrawSphere(const GTransform_QT& TTrans, f32 Radius, uint32_t TColor, int32_t nSeg)
{
    int32_t nStep = nSeg;

    if (nStep < 4)
        nStep = 4;

    if (nStep % 2 == 1)
        nStep++;

    int32_t nVectorCount = (nStep - 1) * nStep * 2 + 2;
    int32_t nTriangleCount = (nStep - 1) * nStep * 2 * 2;
    int32_t nIndexCount = nTriangleCount * 3;
    int32_t nLongitude = nStep * 2; // 经线
    int32_t nLatitude = nStep; // 纬线

    f32    fDelta = GMath::Pi_Two() /  f32(nLongitude);

    std::vector<GVector3>    TSinCos;
    for (int32_t nLoop = 0; nLoop < nLongitude; ++nLoop)
    {
        GVector3 T;
        GMath::SinCos( fDelta * f32(nLoop), T.x, T.y);
        TSinCos.push_back(T);
    }

    GVector3 VLow(GMath::Zero(), GMath::Zero(), -GMath::One());
    GVector3 VHigh(GMath::Zero(), GMath::Zero(), GMath::One());


    GVector3 VSC = TSinCos[nLongitude - 1];

    GVector3 TTLongitude_Last;
    for (int32_t nLoopLongitude = 1; nLoopLongitude < nLatitude; ++nLoopLongitude)
    {
        GVector3& TLongitude = TSinCos[nLoopLongitude];

        GVector3& StartPre = TSinCos[nLongitude - 1];

        GVector3 VLast = TTrans.TransformPosition(GVector3(StartPre.x * TLongitude.x, StartPre.y * TLongitude.x, -TLongitude.y) * Radius);

        for (int32_t nLoopLatitude = 0; nLoopLatitude < nLongitude; ++nLoopLatitude)
        {
            GVector3& TLatitude = TSinCos[nLoopLatitude];

            GVector3 VCurrent = TTrans.TransformPosition(GVector3(TLatitude.x * TLongitude.x, TLatitude.y * TLongitude.x, -TLongitude.y) * Radius);
            DrawLine(VCurrent, VLast, TColor);

            if (nLoopLongitude == 1)
            {
                DrawLine(VCurrent, TTrans.TransformPosition(GVector3(GMath::Zero(), GMath::Zero(), -GMath::One()) * Radius), TColor);
            }
            else
            {
                if (nLoopLongitude == (nLatitude - 1))
                {
                    DrawLine(VCurrent, TTrans.TransformPosition(GVector3(GMath::Zero(), GMath::Zero(), GMath::One()) * Radius), TColor);
                }

                GVector3 VT = TTrans.TransformPosition(GVector3(TLatitude.x * TTLongitude_Last.x, TLatitude.y * TTLongitude_Last.x, -TTLongitude_Last.y) * Radius);
                DrawLine(VCurrent, VT, TColor);
            }

            VLast = VCurrent;
        }

        TTLongitude_Last = TLongitude;
    }
}

void IGLacierDraw::DrawCapsule(const GTransform_QT& TTrans, f32 Radius, f32 HalfHeight, uint32_t TColor, int nSeg)
{
    const f32 AngleIncrement = f32(360) / f32(nSeg);

    GVector3 SeparationDir = GVector3(f32::Zero(), f32::Zero(), f32::One());

    GVector3 VertexPrevious = GVector3(f32::One(), f32::Zero(), f32::Zero());

    GVector3 Center0 = GVector3(f32::Zero(), f32::Zero(), HalfHeight);
    GVector3 Center1 = GVector3(f32::Zero(), f32::Zero(), -HalfHeight);


    for (f32 Angle = AngleIncrement; Angle <= 360.0f; Angle += AngleIncrement)  // iterate over unit circle about capsule's major axis (which is orientation.AxisZ)
    {
        f32 TSin, TCos;
        GMath::SinCos(GMath::DegreesToRadians(Angle), TSin, TCos);

        GVector3 VLocal = GVector3( TCos,  TSin, f32::Zero());

        GVector3 VertexCurrent = VLocal;
        DrawLine(TTrans.TransformPosition(Center0 + VertexCurrent * Radius), TTrans.TransformPosition(Center1 + VertexCurrent * Radius), TColor);  // capsule side segment between spheres
        DrawLine(TTrans.TransformPosition(Center0 + VertexPrevious * Radius), TTrans.TransformPosition(Center0 + VertexCurrent * Radius), TColor);  // cap-circle segment on sphere S0
        DrawLine(TTrans.TransformPosition(Center1 + VertexPrevious * Radius), TTrans.TransformPosition(Center1 + VertexCurrent * Radius), TColor);  // cap-circle segment on sphere S1
        VertexPrevious = VertexCurrent;

        GVector3 VertexPrevious_Longitude = SeparationDir;

        GVector3 VertexCurrentDir = GVector3(TCos, TSin, f32::Zero());

        for (f32 Angle_Longitude = AngleIncrement; Angle_Longitude <= f32(180.0f); Angle_Longitude += AngleIncrement)
        {
            f32 TfSin, TfCos;
            GMath::SinCos(GMath::DegreesToRadians(Angle_Longitude), TfSin, TfCos );

            GVector3 VertexCurrent_Longitude = SeparationDir * TfCos + VertexCurrentDir * TfSin;

            if (Angle_Longitude < (f32(90.0f)))
            {
                if (Angle_Longitude >= (f32(90.0f) - AngleIncrement))
                {
                    DrawLine(TTrans.TransformPosition(VertexCurrent_Longitude * Radius + Center0), TTrans.TransformPosition(Center0 + VertexCurrent * Radius), TColor);
                }

                {
                    DrawLine(TTrans.TransformPosition(VertexCurrent_Longitude * Radius + Center0), TTrans.TransformPosition(VertexPrevious_Longitude * Radius + Center0), TColor);
                }
            }
            else
            {
                if (Angle_Longitude < (f32(90.0f) + AngleIncrement))
                {
                    DrawLine(TTrans.TransformPosition(VertexCurrent_Longitude * Radius + Center1), TTrans.TransformPosition(Center1 + VertexCurrent * Radius), TColor);
                }
                else
                {
                    DrawLine(TTrans.TransformPosition(VertexCurrent_Longitude * Radius + Center1), TTrans.TransformPosition(VertexPrevious_Longitude * Radius + Center1), TColor);
                }
            }

            VertexPrevious_Longitude = VertexCurrent_Longitude;
        }
    }

}



void IGLacierDraw::DrawPlane(const GTransform_QT& TTrans, const GVector3& PlaneNormal, f32 PlaneDis, f32 Size, uint32_t TColor)
{
   /* GTransform_QT Transform;
    Transform.m_Translation =  PlaneNormal  * PlaneDis;
    Transform.SetRotation(FQuat::FindBetweenNormals(FVector(0, 0, 1), Plane.GetNormal()));

    auto P0 = Transform.TransformPosition(FVector(-1, -1, 0) * Size);
    auto P1 = Transform.TransformPosition(FVector(1, -1, 0) * Size);
    auto P2 = Transform.TransformPosition(FVector(1, 1, 0) * Size);
    auto P3 = Transform.TransformPosition(FVector(-1, 1, 0) * Size);

    DrawLine(TTrans.TransformPosition(P0), TTrans.TransformPosition(P1), TColor);
    DrawLine(TTrans.TransformPosition(P1), TTrans.TransformPosition(P2), TColor);
    DrawLine(TTrans.TransformPosition(P2), TTrans.TransformPosition(P3), TColor);
    DrawLine(TTrans.TransformPosition(P3), TTrans.TransformPosition(P0), TColor);
    DrawLine(TTrans.TransformPosition(P0), TTrans.TransformPosition(P2), TColor);
    DrawLine(TTrans.TransformPosition(P1), TTrans.TransformPosition(P3), TColor);
    DrawLine(TTrans.TransformPosition(Transform.GetLocation()), TTrans.TransformPosition(Transform.GetLocation() + Plane.GetNormal() * Size * 0.5f), TColor);
*/
}

