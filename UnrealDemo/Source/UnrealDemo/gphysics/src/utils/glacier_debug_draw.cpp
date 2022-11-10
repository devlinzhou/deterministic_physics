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

#include "glacier_debug_draw.h"
#include "glacier_vector.h"
#include "glacier_plane.h"
#include "glacier_transform_qt.h"
#include <vector>

void IGlacierDraw::DrawBox(const GTransform_QT& TTrans, const GVector3& LocalCenter, const GVector3& HalfSize, GColor TColor)
{
    GVector3 c  = TTrans.TransformPosition( LocalCenter );
    GVector3 d  = HalfSize;
    GVector3 dx = TTrans.TransformNormal( GVector3(d.x, GMath::Zero(), GMath::Zero()));
    GVector3 dy = TTrans.TransformNormal( GVector3(GMath::Zero(), d.y, GMath::Zero()));
    GVector3 dz = TTrans.TransformNormal( GVector3(GMath::Zero(), GMath::Zero(), d.z));

    DrawLine((c + dy + dz - dx), (c + dy + dz + dx), TColor);
    DrawLine((c + dy - dz - dx), (c + dy - dz + dx), TColor);
    DrawLine((c - dy + dz - dx), (c - dy + dz + dx), TColor);
    DrawLine((c - dy - dz - dx), (c - dy - dz + dx), TColor);
    DrawLine((c + dy + dx - dz), (c + dy + dx + dz), TColor);
    DrawLine((c + dy - dx - dz), (c + dy - dx + dz), TColor);
    DrawLine((c - dy + dx - dz), (c - dy + dx + dz), TColor);
    DrawLine((c - dy - dx - dz), (c - dy - dx + dz), TColor);
    DrawLine((c + dz + dx - dy), (c + dz + dx + dy), TColor);
    DrawLine((c + dz - dx - dy), (c + dz - dx + dy), TColor);
    DrawLine((c - dz + dx - dy), (c - dz + dx + dy), TColor);
    DrawLine((c - dz - dx - dy), (c - dz - dx + dy), TColor);
}

void IGlacierDraw::DrawSphere(const GTransform_QT& TTrans, f32 Radius, GColor TColor, int32_t nSeg)
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

void IGlacierDraw::DrawCapsule(const GTransform_QT& TTrans, f32 Radius, f32 HalfHeight, GColor TColor, int nSeg)
{
    const f32 AngleIncrement = f32(360) / f32(nSeg);

    GVector3 SeparationDir = GVector3(GMath::Zero(), GMath::Zero(), GMath::One());

    GVector3 VertexPrevious = GVector3(GMath::One(), GMath::Zero(), GMath::Zero());

    GVector3 Center0 = GVector3(GMath::Zero(), GMath::Zero(), HalfHeight);
    GVector3 Center1 = GVector3(GMath::Zero(), GMath::Zero(), -HalfHeight);


    for (f32 Angle = AngleIncrement; Angle <= GMath::Makef32(360); Angle += AngleIncrement)  // iterate over unit circle about capsule's major axis (which is orientation.AxisZ)
    {
        f32 TSin, TCos;
        GMath::SinCos(GMath::DegreesToRadians(Angle), TSin, TCos);

        GVector3 VLocal = GVector3( TCos,  TSin, GMath::Zero());

        GVector3 VertexCurrent = VLocal;
        DrawLine(TTrans.TransformPosition(Center0 + VertexCurrent * Radius), TTrans.TransformPosition(Center1 + VertexCurrent * Radius), TColor);  // capsule side segment between spheres
        DrawLine(TTrans.TransformPosition(Center0 + VertexPrevious * Radius), TTrans.TransformPosition(Center0 + VertexCurrent * Radius), TColor);  // cap-circle segment on sphere S0
        DrawLine(TTrans.TransformPosition(Center1 + VertexPrevious * Radius), TTrans.TransformPosition(Center1 + VertexCurrent * Radius), TColor);  // cap-circle segment on sphere S1
        VertexPrevious = VertexCurrent;

        GVector3 VertexPrevious_Longitude = SeparationDir;

        GVector3 VertexCurrentDir = GVector3(TCos, TSin, GMath::Zero());

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

void IGlacierDraw::DrawCylinder(const GTransform_QT& TTrans, f32 Radius, f32 HalfHeight, GColor TColor, int nSeg)
{
    const f32 AngleIncrement = f32(360) / f32(nSeg);

    GVector3 SeparationDir = GVector3(GMath::Zero(), GMath::Zero(), GMath::One());

    GVector3 VertexPrevious = GVector3(GMath::One(), GMath::Zero(), GMath::Zero());

    GVector3 Center0 = GVector3(GMath::Zero(), GMath::Zero(), HalfHeight);
    GVector3 Center1 = GVector3(GMath::Zero(), GMath::Zero(), -HalfHeight);


    for (f32 Angle = AngleIncrement; Angle <= GMath::Makef32(360); Angle += AngleIncrement)  // iterate over unit circle about capsule's major axis (which is orientation.AxisZ)
    {
        f32 TSin, TCos;
        GMath::SinCos(GMath::DegreesToRadians(Angle), TSin, TCos);

        GVector3 VLocal = GVector3(TCos, TSin, GMath::Zero());

        GVector3 VertexCurrent = VLocal;
        DrawLine(TTrans.TransformPosition(Center0 + VertexCurrent * Radius), TTrans.TransformPosition(Center1 + VertexCurrent * Radius), TColor);  // capsule side segment between spheres
        DrawLine(TTrans.TransformPosition(Center0 + VertexPrevious * Radius), TTrans.TransformPosition(Center0 + VertexCurrent * Radius), TColor);  // cap-circle segment on sphere S0
        DrawLine(TTrans.TransformPosition(Center1 + VertexPrevious * Radius), TTrans.TransformPosition(Center1 + VertexCurrent * Radius), TColor);  // cap-circle segment on sphere S1
        VertexPrevious = VertexCurrent;

        GVector3 VertexPrevious_Longitude = SeparationDir;

        GVector3 VertexCurrentDir = GVector3(TCos, TSin, GMath::Zero());

        for (f32 Angle_Longitude = AngleIncrement; Angle_Longitude <= f32(180.0f); Angle_Longitude += AngleIncrement)
        {
            f32 TfSin, TfCos;
            GMath::SinCos(GMath::DegreesToRadians(Angle_Longitude), TfSin, TfCos);

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

void IGlacierDraw::DrawPlane(const GTransform_QT& TTrans, const GPlane& TPlane, f32 Size, GColor TColor)
{
    GTransform_QT Transform;

    Transform.m_Pos = -TPlane.m_Normal  * TPlane.GetDistance( GVector3::Zero() );
    Transform.m_Rot = GQuaternion::FindBetweenNormals(GVector3(GMath::One(), GMath::Zero(), GMath::Zero()), TPlane.m_Normal);

    DrawPlane( Transform, Size, TColor);
}

void IGlacierDraw::DrawPlane(const GTransform_QT& TTrans, f32 Size, GColor TColor)
{
    GVector3 V0 = TTrans.TransformNormal(GVector3(GMath::Zero(), GMath::One(), GMath::Zero()));
    GVector3 V1 = TTrans.TransformNormal(GVector3(GMath::Zero(), GMath::Zero(), GMath::One()));

    DrawLine(TTrans.m_Pos,TTrans.m_Pos + TTrans.m_Rot.RotateVector( GVector3::UnitX() * Size * GMath::Makef32(0,15,100)), TColor);

    for (f32 i = -Size; i <= Size; i += GMath::Makef32(10,0,1) )
    {
        GVector3 pt0 = TTrans.m_Pos + V0 * Size + i * V1;
        GVector3 pt1 = TTrans.m_Pos - V0 * Size + i * V1;
        GVector3 pt2 = TTrans.m_Pos + V1 * Size + i * V0;
        GVector3 pt3 = TTrans.m_Pos - V1 * Size + i * V0;

        DrawLine(pt0, pt1, TColor);
        DrawLine(pt2, pt3, TColor);
    }


}

void IGlacierDraw::DrawArrow(const GVector3& V0, const GVector3& VDirection, f32 Size, GColor TColor)
{
    DrawLine( V0, V0 + VDirection * Size, TColor);

    GVector3 VEnd = V0 + VDirection * Size;
    GVector3 VCurrentUp = GVector3::UnitZ();

    if (GVector3::CrossProduct(VDirection, VCurrentUp).SizeSquare() < GMath::Inv_100000())
    {
        VCurrentUp = GVector3::UnitX();
    }

    GVector3 V2 = GVector3::CrossProduct(VDirection, VCurrentUp).GetNormalize();
    GVector3 V3 = GVector3::CrossProduct(V2, VDirection).GetNormalize();

    f32 fWind = GMath::Makef32(0,13,100);
    f32 fLeng = GMath::Makef32(0,25,100);

    DrawLine(V0, VEnd, TColor);

    GVector3 VCenter = VEnd - VDirection * Size * fLeng;

    DrawLine(VEnd, VCenter + V2 * Size * fWind, TColor);
    DrawLine(VEnd, VCenter - V2 * Size * fWind, TColor);

    DrawLine(VEnd, VCenter + V3 * Size * fWind, TColor);
    DrawLine(VEnd, VCenter - V3 * Size * fWind, TColor);
}

