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
#include "glacier_matrix.h"
#include "glacier_plane.h"
#include "glacier_contact.h"

static inline GVector3 Cross_X(f32 a, const GVector3& V)
{
    return GVector3(GMath::Zero(), -a * V.z, a * V.y);
}

static inline GVector3 Cross_Y(f32 a, const GVector3& V)
{
    return  GVector3(a * V.z, GMath::Zero(), -a * V.x);
}

static inline GVector3 Cross_Z(f32 a, const GVector3& V)
{
    return GVector3(-a * V.y, a * V.x, GMath::Zero());
}

static inline GVector3 Cross_AbsX(f32 a, const GVector3& V)
{
    return GVector3(GMath::Zero(), a * V.z, a * V.y);
}

static inline GVector3 Cross_AbsY(f32 a, const GVector3& V)
{
    return  GVector3(a * V.z, GMath::Zero(), a * V.x);
}

static inline GVector3 Cross_AbsZ(f32 a, const GVector3& V)
{
    return GVector3(a * V.y, a * V.x, GMath::Zero());
}



static inline f32 Dot_No_X( const GVector3& SatAxisLocal, const GVector3& HalfExten )
{
    return HalfExten.y * (SatAxisLocal.y) + HalfExten.z * (SatAxisLocal.z);
}

static inline f32 Dot_No_Y(const GVector3& SatAxisLocal, const GVector3& HalfExten)
{
    return HalfExten.x * (SatAxisLocal.x) + HalfExten.z * (SatAxisLocal.z);
}

static inline f32 Dot_No_Z(const GVector3& SatAxisLocal, const GVector3& HalfExten)
{
    return HalfExten.x * (SatAxisLocal.x) + HalfExten.y * (SatAxisLocal.y);
}

bool GCollision_Box::Box_Box(
    const GShapeBox&        ShapA,
    const GTransform_QT&    TransformA,
    const GShapeBox&        ShapB,
    const GTransform_QT&    TransformB,
    GVector3*               pOutPosition,
    GVector3*               pOutNormal)
{
    const GVector3& HalfA = ShapA.HalfExtern;
    const GVector3& HalfB = ShapB.HalfExtern;
    GTransform_QT   B_to_A      = TransformB * TransformA.GetInverse_fast();
    GMatrix3        M_B_To_A( B_to_A.m_Rot );

    GVector3    BLAX      = M_B_To_A.GetRow(0);
    GVector3    BLAY      = M_B_To_A.GetRow(1);
    GVector3    BLAZ      = M_B_To_A.GetRow(2);
    GVector3&   VDisBA    = B_to_A.m_Pos;

    GVector3 VAxis_Abs0 = GVector3(GMath::Abs(BLAX.x), GMath::Abs(BLAY.x), GMath::Abs(BLAZ.x));

    if ((HalfA.x + GVector3::DotProduct( VAxis_Abs0, HalfB) ) < GMath::Abs(VDisBA.x))
        return false;

    GVector3 VAxis_Abs1 = GVector3(GMath::Abs(BLAX.y), GMath::Abs(BLAY.y), GMath::Abs(BLAZ.y));

    if ((HalfA.y + GVector3::DotProduct( VAxis_Abs1, HalfB) ) < GMath::Abs(VDisBA.y))
        return false;

    GVector3 VAxis_Abs2 = GVector3(GMath::Abs(BLAX.z), GMath::Abs(BLAY.z), GMath::Abs(BLAZ.z));

    if ((HalfA.z + GVector3::DotProduct( VAxis_Abs2, HalfB) ) < GMath::Abs(VDisBA.z))
        return false;

    const GVector3& VDisAB = B_to_A.m_Rot.UnRotateVector( -B_to_A.m_Pos );

    const GVector3 VA_LocalB_X = GVector3(VAxis_Abs0.x, VAxis_Abs1.x, VAxis_Abs2.x );
    if ((HalfB.x + GVector3::DotProduct( VA_LocalB_X, HalfA)) < GMath::Abs(VDisAB.x))
        return false;

    const GVector3 VA_LocalB_Y = GVector3(VAxis_Abs0.y, VAxis_Abs1.y, VAxis_Abs2.y);
    if ((HalfB.y + GVector3::DotProduct( VA_LocalB_Y, HalfA)) < GMath::Abs(VDisAB.y))
        return false;

    const GVector3 VA_LocalB_Z = GVector3(VAxis_Abs0.z, VAxis_Abs1.z, VAxis_Abs2.z);
    if ((HalfB.z + GVector3::DotProduct( VA_LocalB_Z, HalfA)) < GMath::Abs(VDisAB.z))
        return false;

    BLAX *= HalfB.x;
    BLAY *= HalfB.y;
    BLAZ *= HalfB.z;

    GVector3 absBLAX = VA_LocalB_X * HalfB.x;
    GVector3 absBLAY = VA_LocalB_Y * HalfB.y;
    GVector3 absBLAZ = VA_LocalB_Z * HalfB.z;

    GVector3 absALBX = VAxis_Abs0 * HalfA.x;
    GVector3 absALBY = VAxis_Abs1 * HalfA.y;
    GVector3 absALBZ = VAxis_Abs2 * HalfA.z;

    if ((Dot_No_X(Cross_AbsX(HalfA.x, absBLAX), HalfA) + Dot_No_X(Cross_AbsX(HalfB.x, absALBX), HalfB)) < GMath::Abs(Dot_No_X(Cross_X(HalfA.x, BLAX), VDisBA)))  return false;
    if ((Dot_No_X(Cross_AbsX(HalfA.x, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBX), HalfB)) < GMath::Abs(Dot_No_X(Cross_X(HalfA.x, BLAY), VDisBA)))  return false;
    if ((Dot_No_X(Cross_AbsX(HalfA.x, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBX), HalfB)) < GMath::Abs(Dot_No_X(Cross_X(HalfA.x, BLAZ), VDisBA)))  return false;

    if ((Dot_No_Y(Cross_AbsY(HalfA.y, absBLAX), HalfA) + Dot_No_X(Cross_AbsX(HalfB.x, absALBY), HalfB)) < GMath::Abs(Dot_No_Y(Cross_Y(HalfA.y, BLAX), VDisBA)))  return false;
    if ((Dot_No_Y(Cross_AbsY(HalfA.y, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBY), HalfB)) < GMath::Abs(Dot_No_Y(Cross_Y(HalfA.y, BLAY), VDisBA)))  return false;
    if ((Dot_No_Y(Cross_AbsY(HalfA.y, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBY), HalfB)) < GMath::Abs(Dot_No_Y(Cross_Y(HalfA.y, BLAZ), VDisBA)))  return false;

    if ((Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAX), HalfA) + Dot_No_X(Cross_AbsX(HalfB.x, absALBZ), HalfB)) < GMath::Abs(Dot_No_Z(Cross_Z(HalfA.z, BLAX), VDisBA)))  return false;
    if ((Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBZ), HalfB)) < GMath::Abs(Dot_No_Z(Cross_Z(HalfA.z, BLAY), VDisBA)))  return false;
    if ((Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBZ), HalfB)) < GMath::Abs(Dot_No_Z(Cross_Z(HalfA.z, BLAZ), VDisBA)))  return false;

    return true;
}


static inline GVector3 BPosStart_X( const GVector3& HalfB, const GTransform_QT& A_to_B )
{
    const GVector3 BEdgeLocal(
        -HalfB.x,
        A_to_B.m_Pos.y > GMath::Zero() ? HalfB.y : -HalfB.y,
        A_to_B.m_Pos.z > GMath::Zero() ? HalfB.z : -HalfB.z);
    return BEdgeLocal;
}

static inline GVector3 BPosStart_Y(const GVector3& HalfB, const GTransform_QT& A_to_B)
{
    const GVector3 BEdgeLocal(
        A_to_B.m_Pos.x > GMath::Zero() ? HalfB.x : -HalfB.x,
        -HalfB.y,
        A_to_B.m_Pos.z > GMath::Zero() ? HalfB.z : -HalfB.z);
    return BEdgeLocal;
}

static inline GVector3 BPosStart_Z(const GVector3& HalfB, const GTransform_QT& A_to_B)
{
    const GVector3 BEdgeLocal(
        A_to_B.m_Pos.x > GMath::Zero() ? HalfB.x : -HalfB.x,
        A_to_B.m_Pos.y > GMath::Zero() ? HalfB.y : -HalfB.y,
        -HalfB.z);
    return BEdgeLocal;
}

static inline GVector3 EdgeInsect_AX( 
    const GVector3& HalfA, 
    const GVector3& HalfB, 
    const GTransform_QT& B_to_A,
    const GTransform_QT& A_to_B,   
    const GVector3& BLA,
    const GVector3& BEdgeLocal,
    const GVector3& OffSet,
    GVector3& Test  )
{
    Test = BEdgeLocal;

    GVector3 VT = B_to_A.TransformPosition(BEdgeLocal) + OffSet;

    f32 Posy = B_to_A.m_Pos.y > GMath::Zero() ? HalfA.y : -HalfA.y;
    return GVector3(
        VT.x + BLA.x * (Posy - VT.y) / BLA.y,
        Posy,
        B_to_A.m_Pos.z > GMath::Zero() ? HalfA.z : -HalfA.z);
}

static inline GVector3 EdgeInsect_AY(
    const GVector3& HalfA,
    const GVector3& HalfB,
    const GTransform_QT& B_to_A,
    const GTransform_QT& A_to_B,
    const GVector3& BLA,
    const GVector3& BEdgeLocal,
    const GVector3& OffSet,
    GVector3& Test )
{
    Test = BEdgeLocal;
    GVector3 VT = B_to_A.TransformPosition(BEdgeLocal) + OffSet;

    f32 Posz = B_to_A.m_Pos.z > GMath::Zero() ? HalfA.z : -HalfA.z;
    return GVector3(
        B_to_A.m_Pos.x > GMath::Zero() ? HalfA.x : -HalfA.x,
        VT.y + BLA.y * (Posz - VT.z) / BLA.z,
        Posz );
}

static inline GVector3 EdgeInsect_AZ(
    const GVector3&         HalfA,
    const GVector3&         HalfB,
    const GTransform_QT&    B_to_A,
    const GTransform_QT&    A_to_B,
    const GVector3&         BLA,
    const GVector3&         BEdgeLocal,
    const GVector3&         OffSet,
    GVector3& Test )
{
    Test = BEdgeLocal;
    GVector3 VT = B_to_A.TransformPosition(BEdgeLocal) + OffSet;

    f32 Posx = B_to_A.m_Pos.x > GMath::Zero() ? HalfA.x : -HalfA.x;
    return GVector3(
        Posx,
        B_to_A.m_Pos.y > GMath::Zero() ? HalfA.y : -HalfA.y,
        VT.z + BLA.z * (Posx - VT.x) / BLA.x);
}

int32_t GCollision_Box::Box_Box_Contact(
    const GShapeBox&        ShapA,
    const GTransform_QT&    TransformA,
    const GShapeBox&        ShapB,
    const GTransform_QT&    TransformB,
    GCollisionContact*      pContact )
{
    const GVector3& HalfA = ShapA.HalfExtern;
    const GVector3& HalfB = ShapB.HalfExtern;
    GTransform_QT   B_to_A = TransformB * TransformA.GetInverse_fast();
    GMatrix3        M_B_To_A(B_to_A.m_Rot);

    GVector3    BLAX = M_B_To_A.GetRow(0);
    GVector3    BLAY = M_B_To_A.GetRow(1);
    GVector3    BLAZ = M_B_To_A.GetRow(2);
    GVector3&   VDisBA = B_to_A.m_Pos;

    f32         Depth[15];
    GVector3    Normals[15];
    GVector3    Pos[15];

    GVector3 VTest[15];

    GVector3 VAxis_Abs0 = GVector3(GMath::Abs(BLAX.x), GMath::Abs(BLAY.x), GMath::Abs(BLAZ.x));
    Depth[0] = (HalfA.x + GVector3::DotProduct(VAxis_Abs0, HalfB)) - GMath::Abs(VDisBA.x); 
    if (Depth[0] < GMath::Zero() )
        return -1;
    Normals[0]  = GVector3( VDisBA.x > GMath::Zero() ? GMath::One() : -GMath::One(), GMath::Zero(), GMath::Zero());

    GVector3 VAxis_Abs1 = GVector3(GMath::Abs(BLAX.y), GMath::Abs(BLAY.y), GMath::Abs(BLAZ.y));
    Depth[1] = (HalfA.y + GVector3::DotProduct(VAxis_Abs1, HalfB)) - GMath::Abs(VDisBA.y);
    if ( Depth[1] < GMath::Zero() )
        return -1;
    Normals[1]  = GVector3( GMath::Zero(), VDisBA.y > GMath::Zero() ? GMath::One() : -GMath::One(), GMath::Zero());

    GVector3 VAxis_Abs2 = GVector3(GMath::Abs(BLAX.z), GMath::Abs(BLAY.z), GMath::Abs(BLAZ.z));
    Depth[2] = (HalfA.z + GVector3::DotProduct(VAxis_Abs2, HalfB)) - GMath::Abs(VDisBA.z);
    if ( Depth[2] < GMath::Zero() )
        return -1;
    Normals[2]  = GVector3(GMath::One(), GMath::Zero(),  VDisBA.z > GMath::Zero() ? GMath::One() : -GMath::One());


    const GVector3& VDisAB = B_to_A.m_Rot.UnRotateVector(-B_to_A.m_Pos);

    const GVector3 VA_LocalB_X = GVector3(VAxis_Abs0.x, VAxis_Abs1.x, VAxis_Abs2.x);
    Depth[3] = (HalfB.x + GVector3::DotProduct(VA_LocalB_X, HalfA)) - GMath::Abs(VDisAB.x);
    if (Depth[3] < GMath::Zero() )
        return -1;
    Normals[3]  = B_to_A.m_Rot.RotateVector( GVector3( VDisAB.x > GMath::Zero() ? GMath::One() : -GMath::One(), GMath::Zero(), GMath::Zero())) ;

    const GVector3 VA_LocalB_Y = GVector3(VAxis_Abs0.y, VAxis_Abs1.y, VAxis_Abs2.y);
    Depth[4] = (HalfB.y + GVector3::DotProduct(VA_LocalB_Y, HalfA)) - GMath::Abs(VDisAB.y);
    if ( Depth[4] < GMath::Zero() )
        return -1;
     Normals[4]  = B_to_A.m_Rot.RotateVector( GVector3( GMath::Zero(), VDisAB.y > GMath::Zero() ? GMath::One() : -GMath::One(), GMath::Zero()) );

    const GVector3 VA_LocalB_Z = GVector3(VAxis_Abs0.z, VAxis_Abs1.z, VAxis_Abs2.z);
    Depth[5] = (HalfB.z + GVector3::DotProduct(VA_LocalB_Z, HalfA)) - GMath::Abs(VDisAB.z);
    if (Depth[4] < GMath::Zero() )
        return -1;
    Normals[5]  = B_to_A.m_Rot.RotateVector( GVector3(GMath::One(), GMath::Zero(),  VDisAB.z > GMath::Zero() ? GMath::One() : -GMath::One()));


    GVector3 sizeBLAX = BLAX * HalfB.x;
    GVector3 sizeBLAY = BLAY * HalfB.y;
    GVector3 sizeBLAZ = BLAZ * HalfB.z;

    GVector3 absBLAX = VA_LocalB_X * HalfB.x;
    GVector3 absBLAY = VA_LocalB_Y * HalfB.y;
    GVector3 absBLAZ = VA_LocalB_Z * HalfB.z;

    GVector3 absALBX = VAxis_Abs0 * HalfA.x;
    GVector3 absALBY = VAxis_Abs1 * HalfA.y;
    GVector3 absALBZ = VAxis_Abs2 * HalfA.z;

    GTransform_QT   A_to_B = B_to_A.GetInverse_fast();

    Normals[6] = Cross_X(HalfA.x, sizeBLAX);
    Depth[6] = (Dot_No_X(Cross_AbsX(HalfA.x, absBLAX), HalfA) + Dot_No_X(Cross_AbsX(HalfB.x, absALBX), HalfB)) - GMath::Abs(Dot_No_X(Normals[6], VDisBA));
    if (Depth[6]  < GMath::Zero() )
        return -1;

    Depth[6] /= Normals[6].Size(); if( Normals[6].y * B_to_A.m_Pos.y < GMath::Zero() ) Normals[6] = -Normals[6];
    Normals[6].Normalize();
    Pos[6] = EdgeInsect_AX( HalfA, HalfB, B_to_A, A_to_B, BLAX, BPosStart_X(HalfB,A_to_B),Normals[6] * Depth[6],  VTest[6]);
        
    Normals[7] = Cross_X(HalfA.x, sizeBLAY);
    Depth[7] = (Dot_No_X(Cross_AbsX(HalfA.x, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBX), HalfB)) - GMath::Abs(Dot_No_X(Normals[7], VDisBA));
    if (Depth[7]  < GMath::Zero())
        return -1;

    Depth[7] /= Normals[7].Size(); if( Normals[7].y * B_to_A.m_Pos.y < GMath::Zero() ) Normals[7] = -Normals[7];
    Normals[7].Normalize();
    Pos[7] = EdgeInsect_AX( HalfA, HalfB, B_to_A, A_to_B, BLAY, BPosStart_Y(HalfB,A_to_B), Normals[7] * Depth[7], VTest[7]);

    Normals[8] = Cross_X(HalfA.x, sizeBLAZ);
    Depth[8] = (Dot_No_X(Cross_AbsX(HalfA.x, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBX), HalfB)) - GMath::Abs(Dot_No_X(Normals[8], VDisBA));
    if (Depth[8]  < GMath::Zero())
        return -1;
    Depth[8] /= Normals[8].Size(); if( Normals[8].y * B_to_A.m_Pos.y < GMath::Zero() ) Normals[8] = -Normals[8];
    Normals[8].Normalize();
    Pos[8] = EdgeInsect_AX( HalfA, HalfB, B_to_A, A_to_B, BLAZ, BPosStart_Z(HalfB,A_to_B), Normals[8] * Depth[8], VTest[8]);


    /////////////////////////////////////////////////////////////////////////////
    Normals[9] = Cross_Y(HalfA.y, sizeBLAX);
    Depth[9] = (Dot_No_Y(Cross_AbsY(HalfA.y, absBLAX), HalfA) + Dot_No_X(Cross_AbsX(HalfB.x, absALBY), HalfB)) - GMath::Abs(Dot_No_Y(Normals[9], VDisBA));
    if (Depth[9]  < GMath::Zero()) 
        return -1;
    Depth[9] /= Normals[9].Size(); if( Normals[9].z * B_to_A.m_Pos.z < GMath::Zero() ) Normals[9] = -Normals[9];
    Normals[9].Normalize();
    Pos[9] = EdgeInsect_AY( HalfA, HalfB, B_to_A, A_to_B, BLAX, BPosStart_X(HalfB,A_to_B), Normals[9] * Depth[9], VTest[9]);

    Normals[10] = Cross_Y(HalfA.y, sizeBLAY);
    Depth[10] = (Dot_No_Y(Cross_AbsY(HalfA.y, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBY), HalfB)) - GMath::Abs(Dot_No_Y(Normals[10], VDisBA));
    if (Depth[10]  < GMath::Zero())
        return -1;
    Depth[10] /= Normals[10].Size(); if( Normals[10].z * B_to_A.m_Pos.z < GMath::Zero() ) Normals[10] = -Normals[10];
    Normals[10].Normalize();
    Pos[10] = EdgeInsect_AY(HalfA, HalfB, B_to_A, A_to_B, BLAY, BPosStart_Y(HalfB,A_to_B), Normals[10] * Depth[10], VTest[10]);

    Normals[11] = Cross_Y(HalfA.y, sizeBLAZ);
    Depth[11] = (Dot_No_Y(Cross_AbsY(HalfA.y, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBY), HalfB)) - GMath::Abs(Dot_No_Y(Normals[11], VDisBA));
    if (Depth[11]  < GMath::Zero()) 
        return -1;
    Depth[11] /= Normals[11].Size(); if( Normals[11].z * B_to_A.m_Pos.z < GMath::Zero() ) Normals[11] = -Normals[11];
    Normals[11].Normalize();
    Pos[11] = EdgeInsect_AY( HalfA, HalfB, B_to_A, A_to_B, BLAZ, BPosStart_Z(HalfB,A_to_B), Normals[11] * Depth[11], VTest[11]);

    /////////////////////////////////////////////////////////////////////////////

    Normals[12] = Cross_Z(HalfA.z, sizeBLAX);
    Depth[12] = (Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAX), HalfA) + Dot_No_X(Cross_AbsX(HalfB.x, absALBZ), HalfB)) - GMath::Abs(Dot_No_Z(Normals[12], VDisBA));
    if (Depth[12]  < GMath::Zero()) 
        return -1;
    Depth[12] /= Normals[12].Size(); if( Normals[12].x * B_to_A.m_Pos.x < GMath::Zero() ) Normals[12] = -Normals[12];
    Normals[12].Normalize();
    Pos[12] = EdgeInsect_AZ( HalfA, HalfB, B_to_A, A_to_B, BLAX, BPosStart_X(HalfB,A_to_B),Normals[12] * Depth[12], VTest[12]);

    Normals[13] = Cross_Z(HalfA.z, sizeBLAY);
    Depth[13] =(Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBZ), HalfB)) - GMath::Abs(Dot_No_Z(Normals[13], VDisBA));
    if (Depth[13]  < GMath::Zero()) 
        return -1;
    Depth[13] /= Normals[13].Size(); if( Normals[13].x * B_to_A.m_Pos.x < GMath::Zero() ) Normals[13] = -Normals[13];
    Normals[13].Normalize();
    Pos[13] = EdgeInsect_AZ(HalfA, HalfB, B_to_A, A_to_B, BLAY, BPosStart_Y(HalfB,A_to_B), Normals[13] * Depth[13], VTest[13]);

    Normals[14] = Cross_Z(HalfA.z, sizeBLAZ);
    Depth[14] =(Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBZ), HalfB)) - GMath::Abs(Dot_No_Z(Normals[14], VDisBA));
    if (Depth[14]  < GMath::Zero()) 
        return -1;
    Depth[14] /= Normals[14].Size(); if( Normals[14].x * B_to_A.m_Pos.x < GMath::Zero() ) Normals[14] = -Normals[14];
    Normals[14].Normalize();
    Pos[14] = EdgeInsect_AZ(HalfA, HalfB, B_to_A, A_to_B, BLAZ, BPosStart_Z(HalfB,A_to_B), Normals[14] * Depth[14], VTest[14]);


    f32 fDot = Depth[0];
    int32_t nDepth = 0;

    for(int i = 1; i < 15; ++i )
    {
        if(Depth[i] > GMath::Zero() && Depth[i] < fDot )
        {
            nDepth = i;
            fDot = Depth[i];
        }
    }

    if(pContact != nullptr)
    {
        if( nDepth < 6)
        {
            pContact->AddContactPoint(
                TransformA.TransformPosition(GVector3::Zero()),
                TransformA.TransformNormal(-Normals[nDepth]).GetNormalize(),
                -fDot);

            pContact->VTest = TransformB.TransformPosition(VTest[nDepth]);
        }
        else if (nDepth >= 6)
        {
            pContact->AddContactPoint(
                TransformA.TransformPosition(Pos[nDepth]),
                TransformA.TransformNormal(-Normals[nDepth]).GetNormalize(),
                -fDot);

            pContact->VTest = TransformB.TransformPosition(GVector3::Zero());
        }

    }



    return nDepth;
}


#define MAX_NB_CTCS	8 + 12*5 + 6*4
#define ABS_GREATER(x, y)			(GMath::Abs(x) > (y))
#define ABS_SMALLER_EQUAL(x, y)		(GMath::Abs(x) <= (y))
#define PXC_IS_NEGATIVE(x)          ((x) < GMath::Zero())


typedef uint32_t    PxU32;
typedef int32_t     PxI32;



enum
{
    AXIS_A0, AXIS_A1, AXIS_A2,
    AXIS_B0, AXIS_B1, AXIS_B2
};

struct VertexInfo
{
    GVector3	pos;
    bool	penetrate;
    bool	area;
};




// face => 4 vertices of a face of the cube (i.e. a quad)
static inline f32 IsInYZ(const f32 y, const f32 z, const VertexInfo** face)
{
    // Warning, indices have been remapped. We're now actually like this:
    //
    // 3+------+2
    //  |   |  |
    //  |   *--|
    //  | (y,z)|
    // 0+------+1
    f32 PreviousY = face[3]->pos.y;
    f32 PreviousZ = face[3]->pos.z;

    // Loop through quad vertices
    for (PxI32 i = 0; i < 4; i++)
    {
        const f32 CurrentY = face[i]->pos.y;
        const f32 CurrentZ = face[i]->pos.z;

        // |CurrentY - PreviousY      y - PreviousY|
        // |CurrentZ - PreviousZ      z - PreviousZ|
        // => similar to backface culling, check each one of the 4 triangles are consistent, in which case
        // the point is within the parallelogram.
        if ((CurrentY - PreviousY) * (z - PreviousZ) - (CurrentZ - PreviousZ) * (y - PreviousY) >= GMath::Zero())
            return -GMath::One();

        PreviousY = CurrentY;
        PreviousZ = CurrentZ;
    }

    f32 x = face[0]->pos.x;
    {
        const f32 ay = y - face[0]->pos.y;
        const f32 az = z - face[0]->pos.z;

        GVector3 b = face[1]->pos - face[0]->pos;	// ### could be precomputed ?
        x += b.x * (ay * b.y + az * b.z) / b.SizeSquare();		// ### could be precomputed ?

        b = face[3]->pos - face[0]->pos;		// ### could be precomputed ?
        x += b.x * (ay * b.y + az * b.z) / b.SizeSquare();		// ### could be precomputed ?
    }

    return x;
}

// Test with respect to the quad defined by (0,-y1,-z1) and (0,y1,z1)
//  +------+ y1    y
//  |      |       |
//  |  *   |       |
//  |      |       |
//  +------+ -y1   *-----z
static PxI32 generateContacts(
	GCollisionContact& contactBuffer,
    const GVector3& contactNormal,
	f32 y1,
    f32 z1,
    const GVector3& box2,
	const GTransform_QT& transform0,
    const GTransform_QT& transform1,
    f32 contactDistance)
{
//	PxI32 NbContacts=0;
	contactBuffer.ClearPoint();
	y1 += contactDistance;
	z1 += contactDistance;

	const GTransform_QT trans1to0 =  transform1 * transform0.GetInverse_fast();

	VertexInfo vtx[8];	// The 8 cube vertices
//	PxI32 i;

	//     6+------+7
	//     /|     /|
	//    / |    / |
	//   / 4+---/--+5
	// 2+------+3 /    y   z
	//  | /    | /     |  /
	//  |/     |/      |/
	// 0+------+1      *---x

    GMatrix3 trans1to0m( trans1to0.m_Rot);

	{
        const GVector3 ex = trans1to0m.GetRow(0) * box2.x;
        const GVector3 ey = trans1to0m.GetRow(1) * box2.y;
        const GVector3 ez = trans1to0m.GetRow(2) * box2.z;
		

		vtx[0].pos = trans1to0.m_Pos - ex - ey - ez;
		vtx[1].pos = trans1to0.m_Pos + ex - ey - ez;
		vtx[2].pos = trans1to0.m_Pos - ex + ey - ez;
		vtx[3].pos = trans1to0.m_Pos + ex + ey - ez;
		vtx[4].pos = trans1to0.m_Pos - ex - ey + ez;
		vtx[5].pos = trans1to0.m_Pos + ex - ey + ez;
		vtx[6].pos = trans1to0.m_Pos - ex + ey + ez;
		vtx[7].pos = trans1to0.m_Pos + ex + ey + ez;

	}

	// Create vertex info for 8 vertices
	for(PxU32 i=0; i<8; i++)
	{
		// Vertex suivant
		VertexInfo& p = vtx[i];
		// test the point with respect to the x = 0 plane
		//		if(p.pos.x < 0)
		if(p.pos.x < -contactDistance)	//if(PXC_IS_NEGATIVE(p.pos.x))
		{
			p.area		= false;
			p.penetrate	= false;
			continue;
		}

		{
			// we penetrated the quad plane
			p.penetrate = true;
			// test to see if we are in the quad
			// PxAbs => thus we test Y with respect to -Y1 and +Y1 (same for Z)
			//			if(PxAbs(p->pos.y) <= y1 && PxAbs(p->pos.z) <= z1)
			if(ABS_SMALLER_EQUAL(p.pos.y, y1) && ABS_SMALLER_EQUAL(p.pos.z, z1))
			{
				// the point is inside the quad
				p.area=true;
				contactBuffer.AddContactPoint(p.pos, contactNormal, -p.pos.x);
			}
			else
			{
				p.area=false;
			}
		}
	}

	// Teste 12 edges on the quad
	static const PxI32 indices[]={ 0,1, 1,3, 3,2, 2,0, 4,5, 5,7, 7,6, 6,4, 0,4, 1,5, 2,6, 3,7, };
	const PxI32* runningLine = indices;
	const PxI32* endLine = runningLine+24;
	while(runningLine!=endLine)
	{
		// The two vertices of the current edge
		const VertexInfo* p1 = &vtx[*runningLine++];
		const VertexInfo* p2 = &vtx[*runningLine++];

		// Penetrate|Area|Penetrate|Area => 16 cases

		// We only take the edges that at least penetrated the quad's plane into account.
		if(p1->penetrate || p2->penetrate)
		{
			// If at least one of the two vertices is not in the quad...
			if(!p1->area || !p2->area)
			{
				// Test y
				if(p1->pos.y > p2->pos.y)	
                { 
                    const VertexInfo* tmp=p1; p1=p2; p2=tmp;
                }
				// Impact on the +Y1 edge of the quad
				if(p1->pos.y < y1 && p2->pos.y >= y1)	
					// => a point under Y1, the other above
				{
					// Case 1
					f32 a = (y1 - p1->pos.y)/(p2->pos.y - p1->pos.y);
					f32 z = p1->pos.z + (p2->pos.z - p1->pos.z)*a;
					if(GMath::Abs(z) <= z1)
					{
						f32 x = p1->pos.x + (p2->pos.x - p1->pos.x)*a;
						if(x+contactDistance>= GMath::Zero())
						{
							contactBuffer.AddContactPoint(GVector3(x, y1, z), contactNormal, -x);
						}
					}
				}
				// Impact on the edge -Y1 of the quad
				if(p1->pos.y < -y1 && p2->pos.y >= -y1)
				{
					// Case 2
					f32 a = (-y1 - p1->pos.y)/(p2->pos.y - p1->pos.y);
					f32 z = p1->pos.z + (p2->pos.z - p1->pos.z)*a;
					if(GMath::Abs(z) <= z1)
					{
						f32 x = p1->pos.x + (p2->pos.x - p1->pos.x)*a;
						if(x+contactDistance >= GMath::Zero())
						{
							contactBuffer.AddContactPoint(GVector3(x, -y1, z), contactNormal, -x);
						}
					}
				}

				// Test z
				if(p1->pos.z > p2->pos.z)		{ const VertexInfo* tmp=p1; p1=p2; p2=tmp; }
				// Impact on the edge +Z1 of the quad
				if(p1->pos.z < z1 && p2->pos.z >= z1)
				{
					// Case 3
					f32 a = (z1 - p1->pos.z)/(p2->pos.z - p1->pos.z);
					f32 y = p1->pos.y + (p2->pos.y - p1->pos.y)*a;
					if(GMath::Abs(y) <= y1)
					{
						f32 x = p1->pos.x + (p2->pos.x - p1->pos.x)*a;
						if(x+contactDistance >= GMath::Zero())
						{
							contactBuffer.AddContactPoint(GVector3(x, y, z1), contactNormal, -x);
						}
					}
				}
				// Impact on the edge -Z1 of the quad
				if(p1->pos.z < -z1 && p2->pos.z >= -z1)
				{
					// Case 4
					f32 a = (-z1 - p1->pos.z)/(p2->pos.z - p1->pos.z);
					f32 y = p1->pos.y + (p2->pos.y - p1->pos.y)*a;
					if(GMath::Abs(y) <= y1)
					{
						f32 x = p1->pos.x + (p2->pos.x - p1->pos.x)*a;
						if(x+contactDistance >= GMath::Zero())
						{
							contactBuffer.AddContactPoint(GVector3(x, y, -z1), contactNormal, -x);
						}
					}
				}
			}

			// The case where one point penetrates the plane, and the other is not in the quad.
			if((!p1->penetrate && !p2->area) || (!p2->penetrate && !p1->area))
			{
				// Case 5
				f32 a = (-p1->pos.x)/(p2->pos.x - p1->pos.x);
				f32 y = p1->pos.y + (p2->pos.y - p1->pos.y)*a;
				if(GMath::Abs(y) <= y1)
				{
					f32 z = p1->pos.z + (p2->pos.z - p1->pos.z)*a;
					if(GMath::Abs(z) <= z1)
					{
						contactBuffer.AddContactPoint(GVector3(GMath::Zero(), y, z), contactNormal, GMath::Zero());
					}
				}
			}

		}
	}

	{
		// 6 quads => 6 faces of the cube
		static const PxI32 face[][4]={ {0,1,3,2}, {1,5,7,3}, {5,4,6,7}, {4,0,2,6}, {2,3,7,6}, {0,4,5,1} };
		PxI32 addflg=0;
		for(PxU32 i=0; i<6 && addflg!=0x0f; i++)
		{
			const PxI32* p = face[i];
			const VertexInfo* q[4];
			if((q[0]=&vtx[p[0]])->penetrate && (q[1]=&vtx[p[1]])->penetrate && (q[2]=&vtx[p[2]])->penetrate && (q[3]=&vtx[p[3]])->penetrate)
			{
				if(!q[0]->area || !q[1]->area || !q[2]->area || !q[3]->area)
				{
					if(!(addflg&1))	{ f32 x = IsInYZ(-y1, -z1, q); if(x>=GMath::Zero())	{ addflg|=1; contactBuffer.AddContactPoint(GVector3(x, -y1, -z1), contactNormal, -x);  } }
					if(!(addflg&2))	{ f32 x = IsInYZ( y1, -z1, q); if(x>=GMath::Zero())	{ addflg|=2; contactBuffer.AddContactPoint(GVector3(x,  y1, -z1), contactNormal, -x);  } }
					if(!(addflg&4))	{ f32 x = IsInYZ(-y1,  z1, q); if(x>=GMath::Zero())	{ addflg|=4; contactBuffer.AddContactPoint(GVector3(x, -y1,  z1), contactNormal, -x);  } }
					if(!(addflg&8))	{ f32 x = IsInYZ( y1,  z1, q); if(x>=GMath::Zero())	{ addflg|=8; contactBuffer.AddContactPoint(GVector3(x,  y1,  z1), contactNormal, -x);  } }
				}
			}
		}
	}


	for(int32_t i=0; i<contactBuffer.m_nPointCount; i++)
		contactBuffer.m_Point[i].m_PosOnSurfaceB_World = transform0.TransformPosition(
            contactBuffer.m_Point[i].m_PosOnSurfaceB_World);	

	return PxI32(contactBuffer.GetPointCount());
}

// use physx contact for test
int32_t GCollision_Box::Box_Box_Contact_PhysX(
    const GShapeBox& ShapA,
    const GTransform_QT& transform0,
    const GShapeBox& ShapB,
    const GTransform_QT& transform1,
    class GCollisionContact* pContact)
{

    f32 aafC[3][3];			// matrix C = A^T B, c_{ij} = Dot(A_i,B_j)
	f32 aafAbsC[3][3];		// |c_{ij}|
	f32 afAD[3];			// Dot(A_i,D)

	f32 d1[6];
	f32 overlap[6];

    f32 contactDistance = GMath::Zero();

	GVector3 kD = transform1.m_Pos - transform0.m_Pos;

    GMatrix3  transform0m(transform0.m_Rot);
    GMatrix3  transform1m(transform1.m_Rot);

    const GVector3& extents0 = ShapA.HalfExtern;
    const GVector3& extents1 = ShapB.HalfExtern;

    const GVector3& axis00 = transform0m.GetRow(0);
    const GVector3& axis01 = transform0m.GetRow(1);
    const GVector3& axis02 = transform0m.GetRow(2);
    const GVector3& axis10 = transform1m.GetRow(0);
    const GVector3& axis11 = transform1m.GetRow(1);
    const GVector3& axis12 = transform1m.GetRow(2);

	// Perform Class I tests

	aafC[0][0]	= GVector3::DotProduct(axis00, axis10);
	aafC[0][1]	= GVector3::DotProduct(axis00, axis11);
	aafC[0][2]	= GVector3::DotProduct(axis00, axis12);
	afAD[0]		= GVector3::DotProduct(axis00, kD);
	aafAbsC[0][0] = GMath::Inv_1000000() + GMath::Abs(aafC[0][0]);
	aafAbsC[0][1] = GMath::Inv_1000000() + GMath::Abs(aafC[0][1]);
	aafAbsC[0][2] = GMath::Inv_1000000() + GMath::Abs(aafC[0][2]);
	d1[AXIS_A0] = afAD[0];
	f32 d0 = extents0.x + extents1.x*aafAbsC[0][0] + extents1.y*aafAbsC[0][1] + extents1.z*aafAbsC[0][2];
	overlap[AXIS_A0] = d0 - GMath::Abs(d1[AXIS_A0]) + contactDistance;
	if(PXC_IS_NEGATIVE(overlap[AXIS_A0]))		return 0;

    aafC[1][0]  = GVector3::DotProduct(axis01, axis10);
    aafC[1][1]  = GVector3::DotProduct(axis01, axis11);
    aafC[1][2]  = GVector3::DotProduct(axis01, axis12);
    afAD[1]     = GVector3::DotProduct(axis01, kD);
	aafAbsC[1][0] = GMath::Inv_1000000() + GMath::Abs(aafC[1][0]);
	aafAbsC[1][1] = GMath::Inv_1000000() + GMath::Abs(aafC[1][1]);
	aafAbsC[1][2] = GMath::Inv_1000000() + GMath::Abs(aafC[1][2]);
	d1[AXIS_A1] = afAD[1];
	d0 = extents0.y + extents1.x*aafAbsC[1][0] + extents1.y*aafAbsC[1][1] + extents1.z*aafAbsC[1][2];
	overlap[AXIS_A1] = d0 - GMath::Abs(d1[AXIS_A1]) + contactDistance;
	if(PXC_IS_NEGATIVE(overlap[AXIS_A1]))		return 0;

	aafC[2][0]	= GVector3::DotProduct(axis02, axis10);
	aafC[2][1]	= GVector3::DotProduct(axis02, axis11);
	aafC[2][2]	= GVector3::DotProduct(axis02, axis12);
	afAD[2]		= GVector3::DotProduct(axis02, kD);
	aafAbsC[2][0] = GMath::Inv_1000000() + GMath::Abs(aafC[2][0]);
	aafAbsC[2][1] = GMath::Inv_1000000() + GMath::Abs(aafC[2][1]);
	aafAbsC[2][2] = GMath::Inv_1000000() + GMath::Abs(aafC[2][2]);
	d1[AXIS_A2] = afAD[2];
	d0 = extents0.z + extents1.x*aafAbsC[2][0] + extents1.y*aafAbsC[2][1] + extents1.z*aafAbsC[2][2];
	overlap[AXIS_A2] = d0 - GMath::Abs(d1[AXIS_A2]) + contactDistance;
	if(PXC_IS_NEGATIVE(overlap[AXIS_A2]))		return 0;

	// Perform Class II tests

	d1[AXIS_B0] = GVector3::DotProduct(axis10, kD);
	d0 = extents1.x + extents0.x*aafAbsC[0][0] + extents0.y*aafAbsC[1][0] + extents0.z*aafAbsC[2][0];
	overlap[AXIS_B0] = d0 - GMath::Abs(d1[AXIS_B0]) + contactDistance;
	if(PXC_IS_NEGATIVE(overlap[AXIS_B0]))		return 0;

	d1[AXIS_B1] = GVector3::DotProduct(axis11, kD);
	d0 = extents1.y + extents0.x*aafAbsC[0][1] + extents0.y*aafAbsC[1][1] + extents0.z*aafAbsC[2][1];
	overlap[AXIS_B1] = d0 - GMath::Abs(d1[AXIS_B1]) + contactDistance;
	if(PXC_IS_NEGATIVE(overlap[AXIS_B1]))		return 0;

	d1[AXIS_B2] = GVector3::DotProduct(axis12, kD);
	d0 = extents1.z + extents0.x*aafAbsC[0][2] + extents0.y*aafAbsC[1][2] + extents0.z*aafAbsC[2][2];
	overlap[AXIS_B2] = d0 - GMath::Abs(d1[AXIS_B2]) + contactDistance;
	if(PXC_IS_NEGATIVE(overlap[AXIS_B2]))		return 0;

	// Perform Class III tests - we don't need to store distances for those ones.
	// We only test those axes when objects are likely to be separated, i.e. when they where previously non-colliding. For stacks, we'll have
	// to do full contact generation anyway, and those tests are useless - so we skip them. This is similar to what I did in Opcode.
	//if(!collisionData)	// separated or first run
	{
		f32 d = afAD[2]*aafC[1][0] - afAD[1]*aafC[2][0];
		d0 = contactDistance + extents0.y*aafAbsC[2][0] + extents0.z*aafAbsC[1][0] + extents1.y*aafAbsC[0][2] + extents1.z*aafAbsC[0][1];
		if(ABS_GREATER(d, d0))	return 0;

		d = afAD[2]*aafC[1][1] - afAD[1]*aafC[2][1];
		d0 = contactDistance + extents0.y*aafAbsC[2][1] + extents0.z*aafAbsC[1][1] + extents1.x*aafAbsC[0][2] + extents1.z*aafAbsC[0][0];
		if(ABS_GREATER(d, d0))	return 0;

		d = afAD[2]*aafC[1][2] - afAD[1]*aafC[2][2];
		d0 = contactDistance + extents0.y*aafAbsC[2][2] + extents0.z*aafAbsC[1][2] + extents1.x*aafAbsC[0][1] + extents1.y*aafAbsC[0][0];
		if(ABS_GREATER(d, d0))	return 0;

		d = afAD[0]*aafC[2][0] - afAD[2]*aafC[0][0];
		d0 = contactDistance + extents0.x*aafAbsC[2][0] + extents0.z*aafAbsC[0][0] + extents1.y*aafAbsC[1][2] + extents1.z*aafAbsC[1][1];
		if(ABS_GREATER(d, d0))	return 0;

		d = afAD[0]*aafC[2][1] - afAD[2]*aafC[0][1];
		d0 = contactDistance + extents0.x*aafAbsC[2][1] + extents0.z*aafAbsC[0][1] + extents1.x*aafAbsC[1][2] + extents1.z*aafAbsC[1][0];
		if(ABS_GREATER(d, d0))	return 0;

		d = afAD[0]*aafC[2][2] - afAD[2]*aafC[0][2];
		d0 = contactDistance + extents0.x*aafAbsC[2][2] + extents0.z*aafAbsC[0][2] + extents1.x*aafAbsC[1][1] + extents1.y*aafAbsC[1][0];
		if(ABS_GREATER(d, d0))	return 0;

		d = afAD[1]*aafC[0][0] - afAD[0]*aafC[1][0];
		d0 = contactDistance + extents0.x*aafAbsC[1][0] + extents0.y*aafAbsC[0][0] + extents1.y*aafAbsC[2][2] + extents1.z*aafAbsC[2][1];
		if(ABS_GREATER(d, d0))	return 0;

		d = afAD[1]*aafC[0][1] - afAD[0]*aafC[1][1];
		d0 = contactDistance + extents0.x*aafAbsC[1][1] + extents0.y*aafAbsC[0][1] + extents1.x*aafAbsC[2][2] + extents1.z*aafAbsC[2][0];
		if(ABS_GREATER(d, d0))	return 0;

		d = afAD[1]*aafC[0][2] - afAD[0]*aafC[1][2];
		d0 = contactDistance + extents0.x*aafAbsC[1][2] + extents0.y*aafAbsC[0][2] + extents1.x*aafAbsC[2][1] + extents1.y*aafAbsC[2][0];
		if(ABS_GREATER(d, d0))	return 0;
	}


	//if(collisionData) // if initialized & not previously separated
	//	overlap[collisionData-1] *= 0.999f;	 // Favorise previous axis  .999 is too little.

	f32	minimum = GMath::Makef32(1000000,0,1);
	PxI32	minIndex = 0;

	for(PxU32 i=AXIS_A0; i<6; i++)
	{
		f32 d = overlap[i];

		if(d>=GMath::Zero() && d<minimum)
        {
            minimum=d;
            minIndex=PxI32(i);
        }		// >=0 !!  otherwise bug at sep = 0
	}

	//collisionData = PxU32(minIndex + 1);	// Leave "0" for separation

	const PxU32 sign = PxU32(PXC_IS_NEGATIVE(d1[minIndex]));

	GTransform_QT trs;

    GMatrix3 trsm;


	GVector3 ctcNrm;

	switch(minIndex)
	{
	default:
		return 0;

	case AXIS_A0:
		if(sign)	
		{
			ctcNrm = axis00;
			trs.m_Rot = transform0.m_Rot;
			trs.m_Pos = transform0.m_Pos - extents0.x*axis00;
		}
		else		
		{
//			*ctcNrm = -*ctcNrm;
			ctcNrm = -axis00;

            trsm.Row(0) = -axis00;
            trsm.Row(1) = -axis01;
            trsm.Row(2) =  axis02;
			trs.m_Pos = transform0.m_Pos + extents0.x*axis00;
            trs.m_Rot = trsm.ToQuat();
		}

		return generateContacts(*pContact, ctcNrm, extents0.y, extents0.z, extents1, trs, transform1, contactDistance);

	case AXIS_A1:

		trsm.Row(2) = axis00;	// Factored out
		if(sign)	
		{
			ctcNrm = axis01;
			trsm.Row(0) = axis01;
			trsm.Row(1) = axis02;
			trs.m_Pos = transform0.m_Pos - extents0.y*axis01;
		}
		else		
		{
	
			ctcNrm = -axis01;
			
			trsm.Row(0) = -axis01;
			trsm.Row(1) = -axis02;
			trs.m_Pos = transform0.m_Pos + extents0.y*axis01;
		}
        trs.m_Rot = trsm.ToQuat();
		return generateContacts(*pContact, ctcNrm, extents0.z, extents0.x, extents1, trs, transform1, contactDistance);

	case AXIS_A2:
		trsm.Row(2) = axis01;	// Factored out

		if(sign)	
		{
			ctcNrm = axis02;
			trsm.Row(0) = axis02;
			trsm.Row(1) = axis00;
			trs.m_Pos = transform0.m_Pos - extents0.z*axis02;
		}
		else		
		{
			ctcNrm = -axis02;
			
			trsm.Row(0) = -axis02;
			trsm.Row(1) = -axis00;
			trs.m_Pos = transform0.m_Pos + extents0.z*axis02;
		}
        trs.m_Rot = trsm.ToQuat();
		return generateContacts(*pContact, ctcNrm, extents0.x, extents0.y, extents1, trs, transform1, contactDistance);

	case AXIS_B0:
		if(sign)	
		{
			ctcNrm = axis10;
			trsm.Row(0) = -axis10;
			trsm.Row(1) = -axis11;
			trsm.Row(2) = axis12;
			trs.m_Pos = transform1.m_Pos + extents1.x*axis10;
            trs.m_Rot = trsm.ToQuat();
		}
		else		
		{
			ctcNrm = -axis10;
			trs.m_Rot = transform1.m_Rot;
			trs.m_Pos = transform1.m_Pos - extents1.x*axis10;
		
		}
		return generateContacts(*pContact, ctcNrm, extents1.y, extents1.z, extents0, trs, transform0, contactDistance);

	case AXIS_B1:
		trsm.Row(2) = axis10;	// Factored out
		if(sign)	
		{
			ctcNrm = axis11;
			trsm.Row(0) = -axis11;
			trsm.Row(1) = -axis12;
			trs.m_Pos = transform1.m_Pos + extents1.y*axis11;
		}
		else		
		{ 
			ctcNrm = -axis11;
			
			trsm.Row(0) = axis11;
			trsm.Row(1) = axis12;
			trsm.Row(2) = axis10;
			trs.m_Pos = transform1.m_Pos - extents1.y*axis11;
		}
        trs.m_Rot = trsm.ToQuat();
		return generateContacts(*pContact, ctcNrm, extents1.z, extents1.x, extents0, trs, transform0, contactDistance);

	case AXIS_B2:
		trsm.Row(2) = axis11;	// Factored out

		if(sign)	
		{
			ctcNrm = axis12;
			trsm.Row(0) = -axis12;
			trsm.Row(1) = -axis10;
			trs.m_Pos = transform1.m_Pos + extents1.z*axis12;
		}
		else		
		{ 
			ctcNrm = -axis12;		
			trsm.Row(0) = axis12;
			trsm.Row(1) = axis10;
			trs.m_Pos = transform1.m_Pos - extents1.z*axis12;
		}
        trs.m_Rot = trsm.ToQuat();
		return generateContacts(*pContact, ctcNrm, extents1.x, extents1.y, extents0, trs, transform0, contactDistance);
	}
}