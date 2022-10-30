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

#include "glacier_math.h"

class GVector2
{
public:
    f32 x;
    f32 y;

    inline explicit constexpr GVector2( const f32 fX, const f32 fY) :x( fX ),y( fY )  { }

};

class GVector3
{
public:
    f32 x;
    f32 y;
    f32 z;

public:
    GVector3(){}
    inline          constexpr GVector3( const GVector3&) = default;
    inline explicit constexpr GVector3( const f32 fX, const f32 fY, const f32 fZ) :x( fX ),y( fY ), z( fZ )  { }
    inline explicit           GVector3( const int32_t fX, const int32_t fY, const int32_t fZ) :x( f32(fX) ),y( f32(fY) ), z( f32(fZ) )  { }
    inline explicit constexpr GVector3( const f32* pf)     : x( pf[0] ),   y( pf[1] ),     z( pf[2] )   { }
    inline explicit constexpr GVector3( const f32 fScaler)  : x( fScaler ), y( fScaler ),   z( fScaler ) { }
 
public:

    inline f32* Ptr(void) { return &x; }
    inline const f32* Ptr(void) const { return &x; }
    inline f32 operator [] ( const uint32_t i ) const { return *( &x + i ); }
    inline f32& operator [] ( const uint32_t i ) { return *( &x + i ); }

    inline GVector3& operator = ( const GVector3& V )
    {
        x = V.x; y = V.y; z = V.z; return *this;
    }

    inline GVector3& operator = ( const f32 fScaler )
    {
        x = fScaler;        y = fScaler;        z = fScaler;
        return *this;
    }

    inline bool operator == ( const GVector3& V ) const
    {
        return ( x == V.x && y == V.y && z == V.z );
    }

    inline bool operator != ( const GVector3& V ) const
    {
        return ( x != V.x || y != V.y || z != V.z );
    }

    GFORCE_INLINE GVector3 operator +( const GVector3& V ) const
    {
        return GVector3( x + V.x, y + V.y, z + V.z );
    }

    GFORCE_INLINE GVector3 operator -( const GVector3& V ) const
    {
        return GVector3( x - V.x, y - V.y, z - V.z );
    }

    GFORCE_INLINE GVector3 operator *( const f32 fScalar ) const
    {
        return GVector3( x * fScalar, y * fScalar, z * fScalar );
    }

    GFORCE_INLINE GVector3 operator *( const GVector3& V ) const
    {
        return GVector3( x * V.x, y * V.y, z * V.z );
    }

    inline GVector3 operator /( const f32 fScalar ) const
    {
        f32 fInv = GMath::One() / fScalar;
        return GVector3( x * fInv, y * fInv, z * fInv );
    }

    inline GVector3 operator /( const GVector3& V ) const
    {
        return GVector3( x / V.x, y / V.y, z / V.z );
    }

    inline const GVector3& operator +( ) const
    {
        return *this;
    }

    inline GVector3 operator -( ) const
    {
        return GVector3( -x, -y, -z );
    }

    inline friend GVector3 operator * ( const f32 fScalar, const GVector3&    V )
    {
        return GVector3( fScalar * V.x, fScalar * V.y, fScalar * V.z );
    }

    inline friend GVector3 operator / ( const f32 fScalar, const GVector3&    V )
    {
        return GVector3( fScalar / V.x,    fScalar / V.y,    fScalar / V.z );
    }

    inline friend GVector3 operator + ( const GVector3& V, const f32 f )
    {
        return GVector3( V.x + f, V.y + f, V.z + f );
    }

    inline friend GVector3 operator + ( const f32 f, const GVector3& V )
    {
        return GVector3 ( f + V.x, f + V.y, f + V.z );
    }

    inline friend GVector3 operator - ( const GVector3& V, const f32 f )
    {
        return GVector3( V.x - f, V.y - f, V.z - f );
    }

    inline friend GVector3 operator - ( const f32 f, const GVector3& V )
    {
        return GVector3 ( f - V.x, f - V.y, f - V.z );
    }

    inline GVector3& operator += ( const GVector3& V )
    {
        x += V.x; y += V.y; z += V.z;
        return *this;
    }

    inline GVector3& operator += ( const f32 f )
    {
        x += f; y += f; z += f;
        return *this;
    }

    inline GVector3& operator -= ( const GVector3& V )
    {
        x -= V.x; y -= V.y; z -= V.z;
        return *this;
    }

    inline GVector3& operator -= ( const f32 f )
    {
        x -= f; y -= f; z -= f;
        return *this;
    }

    inline GVector3& operator *= ( const f32 fScalar )
    {
        x *= fScalar; y *= fScalar; z *= fScalar;
        return *this;
    }

    inline GVector3& operator *= ( const GVector3& V )
    {
        x *= V.x; y *= V.y; z *= V.z;
        return *this;
    }

    inline GVector3& operator /= ( const f32 fScalar )
    {
        f32 fInv = GMath::One() / fScalar;
        x *= fInv; y *= fInv; z *= fInv;
        return *this;
    }

    inline GVector3& operator /= ( const GVector3& V )
    {
        x = x / V.x; y = y / V.y; z = z / V.z;
        return *this;
    }

    inline f32 Size( ) const
    {
        return GMath::Sqrt( x * x + y * y + z * z );
    }

    inline f32 SizeSquare( ) const
    {
        return x * x + y * y + z * z;
    }

    inline f32 Size2D( ) const
    {
        return GMath::Sqrt( x * x + z * z );
    }

    inline f32 Size2DSquare( ) const
    {
        return x * x + z * z;
    }

    static inline f32 Distance(const GVector3& a, const GVector3& V )
    {
        return (a - V).Size( );
    }

    static inline f32 DistanceSquare( const GVector3& a, const GVector3& V )
    {
        return ( a - V ).SizeSquare( );
    }

    static GFORCE_INLINE f32 DotProduct( const GVector3& a, const GVector3& V )
    {
        return a.x * V.x + a.y * V.y + a.z * V.z;
    }

    inline void Normalize( )
    {
        f32 f = SizeSquare();
        if( f < GMath::Epsilon() )
        {
            *this = Zero();
        }
        else
        {
            

            f32 finv = GMath::InvSqrt(f);
            x *= finv;
            y *= finv;
            z *= finv;
        }
    }

    inline GVector3 GetNormalize( ) const
    {
        GVector3 TV = *this;
        TV.Normalize( );
        return TV;
    }

    inline f32 AbsDotProduct( const GVector3& V ) const
    {
        return GMath::Abs( x * V.x ) + GMath::Abs( y * V.y ) + GMath::Abs( z * V.z );
    }

    static inline GVector3 CrossProduct( const GVector3& a, const GVector3& V )
    {
        return GVector3( a.y * V.z - a.z * V.y, a.z * V.x - a.x * V.z, a.x * V.y - a.y * V.x );
    }

    inline bool operator < ( const GVector3& V ) const
    {
        return x < V.x && y < V.y && z < V.z;
    }

    inline bool operator <= ( const GVector3& V ) const
    {
        return x <= V.x && y <= V.y && z <= V.z;
    }

    inline bool operator > ( const GVector3& V ) const
    {
        return x > V.x && y > V.y && z > V.z ;
    }

    inline bool operator >= ( const GVector3& V ) const
    {
        return x >= V.x && y >= V.y && z >= V.z ;
    }


    inline void MakeFloor( const GVector3& V )
    {
        if( V.x < x ) x = V.x;
        if( V.y < y ) y = V.y;
        if( V.z < z ) z = V.z;
    }

    inline void MakeCeil( const GVector3& V )
    {
        if( V.x > x ) x = V.x;
        if( V.y > y ) y = V.y;
        if( V.z > z ) z = V.z;
    }

    inline GVector3 ReflectByNormal( const GVector3& VNormal ) const
    {
        return  *this - VNormal * ( GMath::Two() * DotProduct( *this, VNormal ) );
    }

    static inline GVector3 GetNoParallel( const GVector3& V )
    {
        if( V.x >= V.y && V.x >= V.z)
        {
            return GVector3::UnitY();
        }
        else if(V.y >= V.z )
        {
            return GVector3::UnitZ();
        }
        else
        {
            return GVector3::UnitX();
        }
    }

    inline GVector3 Lerp( const GVector3& V2, f32 fLerp ) const
    {
        return GVector3(
            ( x * ( GMath::One() - fLerp ) ) + ( V2.x * fLerp ),
            ( y * ( GMath::One() - fLerp ) ) + ( V2.y * fLerp ),
            ( z * ( GMath::One() - fLerp ) ) + ( V2.z * fLerp ) );
    }

    inline GVector3 LerpClamp(const GVector3& V2, f32 fLerp) const
    {
        f32 TClamp = GMath::Clamp( fLerp, GMath::Zero(), GMath::One());
        return GVector3(
            (x * (GMath::One() - TClamp)) + (V2.x * TClamp),
            (y * (GMath::One() - TClamp)) + (V2.y * TClamp),
            (z * (GMath::One() - TClamp)) + (V2.z * TClamp));
    }

    inline GVector3 Abs() const {return GVector3( GMath::Abs(x), GMath::Abs(y), GMath::Abs(z));}

    inline f32 Min( )   const { return GMath::Min3(x, y, z); }
    inline f32 Max( )   const { return GMath::Max3(x, y ,z); }
    inline f32 AbsMin( )const { return GMath::Min3(GMath::Abs(x), GMath::Abs(y), GMath::Abs(z)); }
    inline f32 AbsMax( )const { return GMath::Max3(GMath::Abs(x), GMath::Abs(y), GMath::Abs(z)); }

    static inline GVector3 Min(const GVector3& a, const GVector3& b)
    {
        return GVector3(GMath::Min(a.x, b.x), GMath::Min(a.y, b.y), GMath::Min(a.z, b.z));
    }

    static inline GVector3 Max( const GVector3& a, const GVector3& b)
    {
        return GVector3(GMath::Max(a.x, b.x), GMath::Max(a.y, b.y), GMath::Max(a.z, b.z) );
    }

    inline bool IsZero( void ) const 
    { 
        return x == GMath::Zero() && y == GMath::Zero() && z == GMath::Zero();
    }

    inline bool IsNearlyZero( f32 Epsilon = GMath::Epsilon() ) const
    {
        return AbsMax() < Epsilon;
    }


    static inline GVector3 Identity( )  { return GVector3( GMath::One(),  GMath::One(),  GMath::One()); }
    static inline GVector3 Zero( )      { return GVector3( GMath::Zero(), GMath::Zero(), GMath::Zero()); }
    static inline GVector3 UnitX( )     { return GVector3( GMath::One(),  GMath::Zero(), GMath::Zero()); }
    static inline GVector3 UnitY( )     { return GVector3( GMath::Zero(), GMath::One(),  GMath::Zero()); }
    static inline GVector3 UnitZ( )     { return GVector3( GMath::Zero(), GMath::Zero(), GMath::One()); }
    static inline GVector3 NegtiveX( )  { return GVector3(-GMath::One(),  GMath::Zero(), GMath::Zero()); }
    static inline GVector3 NegtiveY( )  { return GVector3( GMath::Zero(),-GMath::One(),  GMath::Zero()); }
    static inline GVector3 NegtiveZ( )  { return GVector3( GMath::Zero(), GMath::Zero(),-GMath::One()); }



    void Test();
};
