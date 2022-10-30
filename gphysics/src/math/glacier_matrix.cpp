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

#include "glacier_matrix.h"

GMatrix3::GMatrix3( const GMatrix4& M4 )
{
    m[0][0] = M4.m[0][0];
    m[0][1] = M4.m[0][1];
    m[0][2] = M4.m[0][2];

    m[1][0] = M4.m[1][0];
    m[1][1] = M4.m[1][1];
    m[1][2] = M4.m[1][2];

    m[2][0] = M4.m[2][0];
    m[2][1] = M4.m[2][1];
    m[2][2] = M4.m[2][2];
}

bool GMatrix3::Inverse( GMatrix3& M3Out, f32 fTolerance ) const
{
    M3Out[0][0] = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    M3Out[0][1] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
    M3Out[0][2] = m[0][1] * m[1][2] - m[0][2] * m[1][1];

    M3Out[1][0] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    M3Out[1][1] = m[0][0] * m[2][2] - m[0][2] * m[2][0];
    M3Out[1][2] = m[0][2] * m[1][0] - m[0][0] * m[1][2];

    M3Out[2][0] = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    M3Out[2][1] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
    M3Out[2][2] = m[0][0] * m[1][1] - m[0][1] * m[1][0];

    f32 fDet = m[0][0] * M3Out[0][0] + m[0][1] * M3Out[1][0] + m[0][2] * M3Out[2][0];

    if( GMath::Abs( fDet ) <= fTolerance )
    {
        return false;
    }

    f32 fInvDet = GMath::One() / fDet;

    M3Out.m[0][0] *= fInvDet;
    M3Out.m[0][1] *= fInvDet;
    M3Out.m[0][2] *= fInvDet;

    M3Out.m[1][0] *= fInvDet;
    M3Out.m[1][1] *= fInvDet;
    M3Out.m[1][2] *= fInvDet;

    M3Out.m[2][0] *= fInvDet;
    M3Out.m[2][1] *= fInvDet;
    M3Out.m[2][2] *= fInvDet;

    return true;
}

inline static f32 SMINOR
(
    const GMatrix4&    M,
    const uint32_t        nR0,
    const uint32_t        nR1, 
    const uint32_t        nR2, 
    const uint32_t        nC0,
    const uint32_t        nC1,
    const uint32_t        nC2
)
{
    return 
        M[nR0][nC0] * ( M[nR1][nC1] * M[nR2][nC2] - M[nR2][nC1] * M[nR1][nC2] ) -
        M[nR0][nC1] * ( M[nR1][nC0] * M[nR2][nC2] - M[nR2][nC0] * M[nR1][nC2] ) +
        M[nR0][nC2] * ( M[nR1][nC0] * M[nR2][nC1] - M[nR2][nC0] * M[nR1][nC1] );
}

GQuaternion GMatrix4::ToQuat( void ) const
{
    f32 fTrace = m[0][0] + m[1][1] + m[2][2];
    f32 fRoot;

    GQuaternion Reuslt = GQuaternion::Identity();
    if (fTrace > 0.f)
    {
        fRoot = GMath::Sqrt(fTrace + GMath::One());
        Reuslt.w    = GMath::Half() * fRoot;
        fRoot       = GMath::Half() / fRoot;
        Reuslt.x    = (m[1][2] - m[2][1]) * fRoot;
        Reuslt.y    = (m[2][0] - m[0][2]) * fRoot;
        Reuslt.z    = (m[0][1] - m[1][0]) * fRoot;
    }
    else
    {
        static uint32_t s_iNext[3] = { 1, 2, 0 };
        uint32_t i = 0;
        if (m[1][1] > m[0][0])
        {
            i = 1;
        }
        if (m[2][2] > m[i][i])
        {
            i = 2;
        }

        uint32_t j = s_iNext[i];
        uint32_t k = s_iNext[j];

        fRoot = GMath::Sqrt(m[i][i] - m[j][j] - m[k][k] + GMath::One());
        f32* apkQuat[3] = { &Reuslt.x, &Reuslt.y, &Reuslt.z };
        *apkQuat[i] = GMath::Half() * fRoot;
        fRoot       = GMath::Half() / fRoot;
        Reuslt.w    = (m[j][k] - m[k][j]) * fRoot;
        *apkQuat[j] = (m[i][j] + m[j][i]) * fRoot;
        *apkQuat[k] = (m[i][k] + m[k][i]) * fRoot;
    }

    return Reuslt;
}

GMatrix4 GMatrix4::Adjoint( void ) const
{
    return GMatrix4(
         SMINOR( *this, 1, 2, 3, 1, 2, 3 ),
        -SMINOR( *this, 0, 2, 3, 1, 2, 3 ),
         SMINOR( *this, 0, 1, 3, 1, 2, 3 ),
        -SMINOR( *this, 0, 1, 2, 1, 2, 3 ),

        -SMINOR( *this, 1, 2, 3, 0, 2, 3 ),
         SMINOR( *this, 0, 2, 3, 0, 2, 3 ),
        -SMINOR( *this, 0, 1, 3, 0, 2, 3 ),
         SMINOR( *this, 0, 1, 2, 0, 2, 3 ),

         SMINOR( *this, 1, 2, 3, 0, 1, 3 ),
        -SMINOR( *this, 0, 2, 3, 0, 1, 3 ),
         SMINOR( *this, 0, 1, 3, 0, 1, 3 ),
        -SMINOR( *this, 0, 1, 2, 0, 1, 3 ),

        -SMINOR( *this, 1, 2, 3, 0, 1, 2 ),
         SMINOR( *this, 0, 2, 3, 0, 1, 2 ),
        -SMINOR( *this, 0, 1, 3, 0, 1, 2 ),
         SMINOR( *this, 0, 1, 2, 0, 1, 2 ) );
}

f32 GMatrix4::Determinant( void ) const
{
    return
        m[0][0] * SMINOR( *this, 1, 2, 3, 1, 2, 3 ) -
        m[0][1] * SMINOR( *this, 1, 2, 3, 0, 2, 3 ) +
        m[0][2] * SMINOR( *this, 1, 2, 3, 0, 1, 3 ) -
        m[0][3] * SMINOR( *this, 1, 2, 3, 0, 1, 2 );
}

f32 GMatrix4::Inverse( )
{
    f32 m00 = m[0][0], m01 = m[0][1], m02 = m[0][2], m03 = m[0][3];
    f32 m10 = m[1][0], m11 = m[1][1], m12 = m[1][2], m13 = m[1][3];
    f32 m20 = m[2][0], m21 = m[2][1], m22 = m[2][2], m23 = m[2][3];
    f32 m30 = m[3][0], m31 = m[3][1], m32 = m[3][2], m33 = m[3][3];

    f32 v0 = m20 * m31 - m21 * m30;
    f32 v1 = m20 * m32 - m22 * m30;
    f32 v2 = m20 * m33 - m23 * m30;
    f32 v3 = m21 * m32 - m22 * m31;
    f32 v4 = m21 * m33 - m23 * m31;
    f32 v5 = m22 * m33 - m23 * m32;

    f32 t00 =  ( v5 * m11 - v4 * m12 + v3 * m13 );
    f32 t10 = -( v5 * m10 - v2 * m12 + v1 * m13 );
    f32 t20 =  ( v4 * m10 - v2 * m11 + v0 * m13 );
    f32 t30 = -( v3 * m10 - v1 * m11 + v0 * m12 );

    f32 fDet = ( t00 * m00 + t10 * m01 + t20 * m02 + t30 * m03 );

    if( fDet == 0.f )
    {
        *this = Identity( );
        return GMath::Zero();
    }
    else
    {
        f32 fInvDet = GMath::One() / fDet ;

        f32 d00 = t00 * fInvDet;
        f32 d10 = t10 * fInvDet;
        f32 d20 = t20 * fInvDet;
        f32 d30 = t30 * fInvDet;

        f32 d01 = - ( v5 * m01 - v4 * m02 + v3 * m03 ) * fInvDet;
        f32 d11 = ( v5 * m00 - v2 * m02 + v1 * m03 ) * fInvDet;
        f32 d21 = - ( v4 * m00 - v2 * m01 + v0 * m03 ) * fInvDet;
        f32 d31 = ( v3 * m00 - v1 * m01 + v0 * m02 ) * fInvDet;

        v0 = m10 * m31 - m11 * m30;
        v1 = m10 * m32 - m12 * m30;
        v2 = m10 * m33 - m13 * m30;
        v3 = m11 * m32 - m12 * m31;
        v4 = m11 * m33 - m13 * m31;
        v5 = m12 * m33 - m13 * m32;

        f32 d02 =  ( v5 * m01 - v4 * m02 + v3 * m03 ) * fInvDet;
        f32 d12 = -( v5 * m00 - v2 * m02 + v1 * m03 ) * fInvDet;
        f32 d22 =  ( v4 * m00 - v2 * m01 + v0 * m03 ) * fInvDet;
        f32 d32 = -( v3 * m00 - v1 * m01 + v0 * m02 ) * fInvDet;

        v0 = m21 * m10 - m20 * m11;
        v1 = m22 * m10 - m20 * m12;
        v2 = m23 * m10 - m20 * m13;
        v3 = m22 * m11 - m21 * m12;
        v4 = m23 * m11 - m21 * m13;
        v5 = m23 * m12 - m22 * m13;

        f32 d03 = -( v5 * m01 - v4 * m02 + v3 * m03 ) * fInvDet;
        f32 d13 =  ( v5 * m00 - v2 * m02 + v1 * m03 ) * fInvDet;
        f32 d23 = -( v4 * m00 - v2 * m01 + v0 * m03 ) * fInvDet;
        f32 d33 =  ( v3 * m00 - v1 * m01 + v0 * m02 ) * fInvDet;

        *this = GMatrix4(
            d00, d01, d02, d03,
            d10, d11, d12, d13,
            d20, d21, d22, d23,
            d30, d31, d32, d33 );

        return fDet;
    }
}


void GMatrix4::MakeTransformMatrix(const GVector3& V3Scale, const GQuaternion& Q, const GVector3& VTranslation)
{
    *this = GMatrix3( Q );

    m[0][3] = GMath::Zero();
    m[1][3] = GMath::Zero();
    m[2][3] = GMath::Zero();

    m[3][0] = VTranslation.x;
    m[3][1] = VTranslation.y;
    m[3][2] = VTranslation.z;
    m[3][3] = GMath::One();

    *(GVector3*) operator[](0) *= V3Scale.x;
    *(GVector3*) operator[](1) *= V3Scale.y;
    *(GVector3*) operator[](2) *= V3Scale.z;
}

GMatrix4 GMatrix4::TransposeAdjoint( void ) const
{
    GMatrix4 M4temp;

    M4temp.m[0][0] = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    M4temp.m[0][1] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    M4temp.m[0][2] = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    M4temp.m[0][3] = GMath::Zero();

    M4temp.m[1][0] = m[2][1] * m[0][2] - m[2][2] * m[0][1];
    M4temp.m[1][1] = m[2][2] * m[0][0] - m[2][0] * m[0][2];
    M4temp.m[1][2] = m[2][0] * m[0][1] - m[2][1] * m[0][0];
    M4temp.m[1][3] = GMath::Zero();

    M4temp.m[2][0] = m[0][1] * m[1][2] - m[0][2] * m[1][1];
    M4temp.m[2][1] = m[0][2] * m[1][0] - m[0][0] * m[1][2];
    M4temp.m[2][2] = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    M4temp.m[2][3] = GMath::Zero();

    M4temp.m[3][0] = GMath::Zero();
    M4temp.m[3][1] = GMath::Zero();
    M4temp.m[3][2] = GMath::Zero();
    M4temp.m[3][3] = GMath::One();

    return M4temp;
}


void GMatrix4::RemoveScaling( void )
{
    GVector3* pV3;

    pV3 = ( GVector3* ) operator[]( 0 ); 
    pV3->Normalize( );

    pV3 = ( GVector3* ) operator[]( 1 ); 
    pV3->Normalize( );

    pV3 = ( GVector3* ) operator[]( 2 ); 
    pV3->Normalize( );
}


