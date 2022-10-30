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
    GMatrix3        M_B_To_A( B_to_A.m_Rotate );

    GVector3    BLAX      = M_B_To_A.GetRow(0);
    GVector3    BLAY      = M_B_To_A.GetRow(1);
    GVector3    BLAZ      = M_B_To_A.GetRow(2);
    GVector3&   VDisBA    = B_to_A.m_Translation;

    GVector3 VAxis_Abs0 = GVector3(GMath::Abs(BLAX.x), GMath::Abs(BLAY.x), GMath::Abs(BLAZ.x));

    if ((HalfA.x + GVector3::DotProduct( VAxis_Abs0, HalfB) ) < GMath::Abs(VDisBA.x))
        return false;

    GVector3 VAxis_Abs1 = GVector3(GMath::Abs(BLAX.y), GMath::Abs(BLAY.y), GMath::Abs(BLAZ.y));

    if ((HalfA.y + GVector3::DotProduct( VAxis_Abs1, HalfB) ) < GMath::Abs(VDisBA.y))
        return false;

    GVector3 VAxis_Abs2 = GVector3(GMath::Abs(BLAX.z), GMath::Abs(BLAY.z), GMath::Abs(BLAZ.z));

    if ((HalfA.z + GVector3::DotProduct( VAxis_Abs2, HalfB) ) < GMath::Abs(VDisBA.z))
        return false;

    const GVector3& VDisAB = B_to_A.m_Rotate.UnRotateVector( -B_to_A.m_Translation );

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
        A_to_B.m_Translation.y > GMath::Zero() ? HalfB.y : -HalfB.y,
        A_to_B.m_Translation.z > GMath::Zero() ? HalfB.z : -HalfB.z);
    return BEdgeLocal;
}

static inline GVector3 BPosStart_Y(const GVector3& HalfB, const GTransform_QT& A_to_B)
{
    const GVector3 BEdgeLocal(
        A_to_B.m_Translation.x > GMath::Zero() ? HalfB.x : -HalfB.x,
        -HalfB.y,
        A_to_B.m_Translation.z > GMath::Zero() ? HalfB.z : -HalfB.z);
    return BEdgeLocal;
}

static inline GVector3 BPosStart_Z(const GVector3& HalfB, const GTransform_QT& A_to_B)
{
    const GVector3 BEdgeLocal(
        A_to_B.m_Translation.x > GMath::Zero() ? HalfB.x : -HalfB.x,
        A_to_B.m_Translation.y > GMath::Zero() ? HalfB.y : -HalfB.y,
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

    f32 Posy = B_to_A.m_Translation.y > GMath::Zero() ? HalfA.y : -HalfA.y;
    return GVector3(
        VT.x + BLA.x * (Posy - VT.y) / BLA.y,
        Posy,
        B_to_A.m_Translation.z > GMath::Zero() ? HalfA.z : -HalfA.z);
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

    f32 Posz = B_to_A.m_Translation.z > GMath::Zero() ? HalfA.z : -HalfA.z;
    return GVector3(
        B_to_A.m_Translation.x > GMath::Zero() ? HalfA.x : -HalfA.x,
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

    f32 Posx = B_to_A.m_Translation.x > GMath::Zero() ? HalfA.x : -HalfA.x;
    return GVector3(
        Posx,
        B_to_A.m_Translation.y > GMath::Zero() ? HalfA.y : -HalfA.y,
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
    GMatrix3        M_B_To_A(B_to_A.m_Rotate);

    GVector3    BLAX = M_B_To_A.GetRow(0);
    GVector3    BLAY = M_B_To_A.GetRow(1);
    GVector3    BLAZ = M_B_To_A.GetRow(2);
    GVector3&   VDisBA = B_to_A.m_Translation;

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


    const GVector3& VDisAB = B_to_A.m_Rotate.UnRotateVector(-B_to_A.m_Translation);

    const GVector3 VA_LocalB_X = GVector3(VAxis_Abs0.x, VAxis_Abs1.x, VAxis_Abs2.x);
    Depth[3] = (HalfB.x + GVector3::DotProduct(VA_LocalB_X, HalfA)) - GMath::Abs(VDisAB.x);
    if (Depth[3] < GMath::Zero() )
        return -1;
    Normals[3]  = B_to_A.m_Rotate.RotateVector( GVector3( VDisAB.x > GMath::Zero() ? GMath::One() : -GMath::One(), GMath::Zero(), GMath::Zero())) ;

    const GVector3 VA_LocalB_Y = GVector3(VAxis_Abs0.y, VAxis_Abs1.y, VAxis_Abs2.y);
    Depth[4] = (HalfB.y + GVector3::DotProduct(VA_LocalB_Y, HalfA)) - GMath::Abs(VDisAB.y);
    if ( Depth[4] < GMath::Zero() )
        return -1;
     Normals[4]  = B_to_A.m_Rotate.RotateVector( GVector3( GMath::Zero(), VDisAB.y > GMath::Zero() ? GMath::One() : -GMath::One(), GMath::Zero()) );

    const GVector3 VA_LocalB_Z = GVector3(VAxis_Abs0.z, VAxis_Abs1.z, VAxis_Abs2.z);
    Depth[5] = (HalfB.z + GVector3::DotProduct(VA_LocalB_Z, HalfA)) - GMath::Abs(VDisAB.z);
    if (Depth[4] < GMath::Zero() )
        return -1;
    Normals[5]  = B_to_A.m_Rotate.RotateVector( GVector3(GMath::One(), GMath::Zero(),  VDisAB.z > GMath::Zero() ? GMath::One() : -GMath::One()));


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

    Depth[6] /= Normals[6].Size(); if( Normals[6].y * B_to_A.m_Translation.y < GMath::Zero() ) Normals[6] = -Normals[6];
    Normals[6].Normalize();
    Pos[6] = EdgeInsect_AX( HalfA, HalfB, B_to_A, A_to_B, BLAX, BPosStart_X(HalfB,A_to_B),Normals[6] * Depth[6],  VTest[6]);
        
    Normals[7] = Cross_X(HalfA.x, sizeBLAY);
    Depth[7] = (Dot_No_X(Cross_AbsX(HalfA.x, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBX), HalfB)) - GMath::Abs(Dot_No_X(Normals[7], VDisBA));
    if (Depth[7]  < GMath::Zero())
        return -1;

    Depth[7] /= Normals[7].Size(); if( Normals[7].y * B_to_A.m_Translation.y < GMath::Zero() ) Normals[7] = -Normals[7];
    Normals[7].Normalize();
    Pos[7] = EdgeInsect_AX( HalfA, HalfB, B_to_A, A_to_B, BLAY, BPosStart_Y(HalfB,A_to_B), Normals[7] * Depth[7], VTest[7]);

    Normals[8] = Cross_X(HalfA.x, sizeBLAZ);
    Depth[8] = (Dot_No_X(Cross_AbsX(HalfA.x, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBX), HalfB)) - GMath::Abs(Dot_No_X(Normals[8], VDisBA));
    if (Depth[8]  < GMath::Zero())
        return -1;
    Depth[8] /= Normals[8].Size(); if( Normals[8].y * B_to_A.m_Translation.y < GMath::Zero() ) Normals[8] = -Normals[8];
    Normals[8].Normalize();
    Pos[8] = EdgeInsect_AX( HalfA, HalfB, B_to_A, A_to_B, BLAZ, BPosStart_Z(HalfB,A_to_B), Normals[8] * Depth[8], VTest[8]);


    /////////////////////////////////////////////////////////////////////////////
    Normals[9] = Cross_Y(HalfA.y, sizeBLAX);
    Depth[9] = (Dot_No_Y(Cross_AbsY(HalfA.y, absBLAX), HalfA) + Dot_No_X(Cross_AbsX(HalfB.x, absALBY), HalfB)) - GMath::Abs(Dot_No_Y(Normals[9], VDisBA));
    if (Depth[9]  < GMath::Zero()) 
        return -1;
    Depth[9] /= Normals[9].Size(); if( Normals[9].z * B_to_A.m_Translation.z < GMath::Zero() ) Normals[9] = -Normals[9];
    Normals[9].Normalize();
    Pos[9] = EdgeInsect_AY( HalfA, HalfB, B_to_A, A_to_B, BLAX, BPosStart_X(HalfB,A_to_B), Normals[9] * Depth[9], VTest[9]);

    Normals[10] = Cross_Y(HalfA.y, sizeBLAY);
    Depth[10] = (Dot_No_Y(Cross_AbsY(HalfA.y, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBY), HalfB)) - GMath::Abs(Dot_No_Y(Normals[10], VDisBA));
    if (Depth[10]  < GMath::Zero())
        return -1;
    Depth[10] /= Normals[10].Size(); if( Normals[10].z * B_to_A.m_Translation.z < GMath::Zero() ) Normals[10] = -Normals[10];
    Normals[10].Normalize();
    Pos[10] = EdgeInsect_AY(HalfA, HalfB, B_to_A, A_to_B, BLAY, BPosStart_Y(HalfB,A_to_B), Normals[10] * Depth[10], VTest[10]);

    Normals[11] = Cross_Y(HalfA.y, sizeBLAZ);
    Depth[11] = (Dot_No_Y(Cross_AbsY(HalfA.y, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBY), HalfB)) - GMath::Abs(Dot_No_Y(Normals[11], VDisBA));
    if (Depth[11]  < GMath::Zero()) 
        return -1;
    Depth[11] /= Normals[11].Size(); if( Normals[11].z * B_to_A.m_Translation.z < GMath::Zero() ) Normals[11] = -Normals[11];
    Normals[11].Normalize();
    Pos[11] = EdgeInsect_AY( HalfA, HalfB, B_to_A, A_to_B, BLAZ, BPosStart_Z(HalfB,A_to_B), Normals[11] * Depth[11], VTest[11]);

    /////////////////////////////////////////////////////////////////////////////

    Normals[12] = Cross_Z(HalfA.z, sizeBLAX);
    Depth[12] = (Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAX), HalfA) + Dot_No_X(Cross_AbsX(HalfB.x, absALBZ), HalfB)) - GMath::Abs(Dot_No_Z(Normals[12], VDisBA));
    if (Depth[12]  < GMath::Zero()) 
        return -1;
    Depth[12] /= Normals[12].Size(); if( Normals[12].x * B_to_A.m_Translation.x < GMath::Zero() ) Normals[12] = -Normals[12];
    Normals[12].Normalize();
    Pos[12] = EdgeInsect_AZ( HalfA, HalfB, B_to_A, A_to_B, BLAX, BPosStart_X(HalfB,A_to_B),Normals[12] * Depth[12], VTest[12]);

    Normals[13] = Cross_Z(HalfA.z, sizeBLAY);
    Depth[13] =(Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAY), HalfA) + Dot_No_Y(Cross_AbsY(HalfB.y, absALBZ), HalfB)) - GMath::Abs(Dot_No_Z(Normals[13], VDisBA));
    if (Depth[13]  < GMath::Zero()) 
        return -1;
    Depth[13] /= Normals[13].Size(); if( Normals[13].x * B_to_A.m_Translation.x < GMath::Zero() ) Normals[13] = -Normals[13];
    Normals[13].Normalize();
    Pos[13] = EdgeInsect_AZ(HalfA, HalfB, B_to_A, A_to_B, BLAY, BPosStart_Y(HalfB,A_to_B), Normals[13] * Depth[13], VTest[13]);

    Normals[14] = Cross_Z(HalfA.z, sizeBLAZ);
    Depth[14] =(Dot_No_Z(Cross_AbsZ(HalfA.z, absBLAZ), HalfA) + Dot_No_Z(Cross_AbsZ(HalfB.z, absALBZ), HalfB)) - GMath::Abs(Dot_No_Z(Normals[14], VDisBA));
    if (Depth[14]  < GMath::Zero()) 
        return -1;
    Depth[14] /= Normals[14].Size(); if( Normals[14].x * B_to_A.m_Translation.x < GMath::Zero() ) Normals[14] = -Normals[14];
    Normals[14].Normalize();
    Pos[14] = EdgeInsect_AZ(HalfA, HalfB, B_to_A, A_to_B, BLAZ, BPosStart_Z(HalfB,A_to_B), Normals[14] * Depth[14], VTest[14]);


    f32 fDot = Depth[0];
    int32_t nDepth = 0;

    for(int i = 1; i < 15; ++i )
    {
        if( Depth[i] < fDot )
        {
            nDepth = i;
            fDot = Depth[i];
        }
    }

    if( nDepth >= 6 && pContact != nullptr )
    {
        pContact->AddContactPoint( 
            TransformA.TransformPosition(Pos[nDepth]), 
            TransformA.TransformNormal( -Normals[nDepth]).GetNormalize(),
            -fDot);

        pContact->VTest = TransformB.TransformPosition( VTest[nDepth] );
    }


    return nDepth;
}