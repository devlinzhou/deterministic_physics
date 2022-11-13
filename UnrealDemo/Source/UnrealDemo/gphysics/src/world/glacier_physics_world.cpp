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
#include "glacier_physics_world.h"
#include "glacier_vector.h"
#include "glacier_transform_qt.h"
#include "glacier_collision_shape.h"
#include "glacier_physics_utils.h"
#include "glacier_debug_draw.h"
#include "glacier_collision_object.h"
#include "glacier_rigid_body.h"
#include "glacier_time.h"

GGridPosition s_BroadPhaseNeighbourt[13] =
{
    GGridPosition(1, -1, -1),
    GGridPosition(1,  0, -1),
    GGridPosition(1,  1, -1),
    GGridPosition(1, -1,  0),
    GGridPosition(1,  0,  0),
    GGridPosition(1,  1,  0),
    GGridPosition(1, -1,  1),
    GGridPosition(1,  0,  1),
    GGridPosition(1,  1,  1),
    GGridPosition(0,  1, -1),
    GGridPosition(0,  1,  0),
    GGridPosition(0,  1,  1),
    GGridPosition(0,  0,  1)
};

void GGridCell::DebugDraw(IGlacierDraw* pDraw, uint32_t mask) const
{
    if( mask & GPDraw_CeilBox)
        pDraw->DrawBox( GTransform_QT::Identity(), GetCenter(), GetHalfSize(), GColor::Gray() );

    for (int32_t i = 0; i < (int32_t)m_Objects.size(); ++i)
    {
        const GCObject* pObject = m_Objects[i];

        GPhysicsWorld::DebugDrawObject( pDraw, pObject, mask );
    }
}

bool GGridCell::AddCollisionObject(GCObject* pObject)
{
    if (std::find(m_Objects.begin(), m_Objects.end(), pObject) != m_Objects.end())
    {
        return false;
    }

    m_Objects.push_back(pObject);
    
    return true;
}

bool GGridCell::RemoveObject(GCObject* pObject)
{
    std::vector<GCObject*>::iterator iter = std::find(m_Objects.begin(), m_Objects.end(), pObject);
    if (iter != m_Objects.end())
    {
        m_Objects.erase(iter);

        return true;
    }

    return false;
}

bool GPhysicsWorld::AddCollisionObject(GCObject* pObject)
{
    if (std::find(m_Objects.begin(), m_Objects.end(), pObject) != m_Objects.end())
    {
        return false;
    }

    m_Objects.push_back(pObject);
    m_ObjectMap[pObject->GetId()] = pObject;

    GVector3 VPos = pObject->m_Transform.m_Pos;

    GGridPosition CellPos;

    CellPos.x = GMath::FloorToInt(VPos.x / m_nCellWide);
    CellPos.y = GMath::FloorToInt(VPos.y / m_nCellWide);
    CellPos.z = GMath::FloorToInt(VPos.z / m_nCellHeight);

    GGridCell* pCell = nullptr;

    std::map<GGridPosition, GGridCell*>::iterator iter = m_Grids.find(CellPos);

    if (iter != m_Grids.end())
    {
        pCell = iter->second;
    }
    else
    {
        pCell = new GGridCell(CellPos, m_nCellWide, m_nCellHeight);
        m_Grids[CellPos] = pCell;
    }

    if (pCell != nullptr)
    {
        pCell->AddCollisionObject(pObject);

        pObject->m_pGridCell = pCell;
    }

    return true;
}

bool GPhysicsWorld::DeleteCollisionObject( GCObject* pObject )
{
    auto iter = std::find(m_Objects.begin(), m_Objects.end(), pObject);

    if ( iter == m_Objects.end())
    {
        return false;
    }

    m_Objects.erase( iter);
    m_ObjectMap.erase( pObject->GetId() );

    if( pObject->m_pGridCell != nullptr )
    {
        pObject->m_pGridCell->RemoveObject( pObject );
    }

    return true;
}

const GCObject* GPhysicsWorld::FindCollisionObject(uint32_t id) const
{
    auto iter = m_ObjectMap.find( id );
    if( iter != m_ObjectMap.end() )
    {
        return iter->second;
    }


    for (int32_t i = 0; i < (int32_t)m_StaticLargeObj.size(); ++i)
    {
        GCObject* pObject = m_StaticLargeObj[i];

        if( pObject->GetId() == id )
        {
            return pObject;
        }
    }


    return nullptr;
}

bool GPhysicsWorld::AddStaticLargeObj(GCObject* pObject )
{
    if (std::find(m_StaticLargeObj.begin(), m_StaticLargeObj.end(), pObject) != m_StaticLargeObj.end())
    {
        return false;
    }

    m_StaticLargeObj.push_back(pObject);

    return true;
}

bool GPhysicsWorld::DeleteStaticLargeObj( GCObject* pObject )
{
    auto iter = std::find(m_StaticLargeObj.begin(), m_StaticLargeObj.end(), pObject);

    if (iter == m_StaticLargeObj.end())
    {
        return false;
    }

    m_StaticLargeObj.erase(iter);

    return true;
}


bool GPhysicsWorld::UpdateCollisionObject(GCObject* pObject)
{
    GVector3 VPos = pObject->m_Transform.m_Pos;

    GGridPosition newCellPos = GetGridPos(VPos);

    if( pObject->m_pGridCell == nullptr || ( pObject->m_pGridCell != nullptr && pObject->m_pGridCell->m_pos != newCellPos ) )
    {
        GGridCell* pNewCell = nullptr;

        std::map<GGridPosition, GGridCell*>::iterator iter = m_Grids.find(newCellPos);

        if (iter != m_Grids.end())
        {
            pNewCell = iter->second;
        }
        else
        {
            pNewCell = new GGridCell(newCellPos, m_nCellWide, m_nCellHeight);
            m_Grids[newCellPos] = pNewCell;
        }

        pNewCell->AddCollisionObject( pObject );

        if (pObject->m_pGridCell != nullptr)
        {
            pObject->m_pGridCell->RemoveObject(pObject);
        }

        pObject->m_pGridCell = pNewCell;
    }

    return true;
}

void GPhysicsWorld::PreTick()
{
    GPRORILER_FUN
}


void GPhysicsWorld::Tick(f32 DetltaTime)
{
    GPRORILER_FUN
    
    PreTick();

    Simulate(DetltaTime);

    CollisionBroadPhase();

    CollisionNarrowPhase();

    SolveContactConstraint();

    UpdateSceneGrid();
 
    PostTick();
}

void GPhysicsWorld::PostTick()
{
    GTimeProfiler::DebugOut();
}

void GPhysicsWorld::Simulate( f32 DetltaTime )
{
    for (int32_t i = 0; i < (int32_t)m_Objects.size(); ++i)
    {
        GCObject* pObject = m_Objects[i];
        if (pObject->m_CollisionType == ECollisionObjectType::CO_Rigid_Body)
        {
            GRigidBody* pDynamicRigid = (GRigidBody*)pObject;

            if( pDynamicRigid->m_bDynamic )
            {
                pDynamicRigid->Tick_PreTransform(DetltaTime);
                pDynamicRigid->UpdateAABB();
            }
        }
    }


}

void GPhysicsWorld::DebugDrawObject( IGlacierDraw* pDraw, const GCObject* pObj, uint32_t mask )
{
    const GCObject* pObject = pObj;

    if (mask & GPDraw_Shape)
    {

        GColor TColor = GColor::Yellow(); 
        if( pObj->m_ContactArray.size() != 0 )
        {
            TColor = GColor::Red(); 
        }

        GPhyscsUtils::DrawShape(pObject->m_Transform, pObject->m_Shape, pDraw, TColor);
    }
        

    if (mask & GPDraw_LocalBox)
    {
        GAABB TAABB = pObject->GetLocalAABB();
        pDraw->DrawBox(pObject->m_Transform, TAABB.GetCenter(), TAABB.GetHalfSize(), GColor::White());
    }

    if (mask & GPDraw_WorldBox)
    {
        GAABB TAABB = pObject->GetAABB();
        pDraw->DrawBox(GTransform_QT::Identity(), TAABB.GetCenter(), TAABB.GetHalfSize(), GColor::Gray());
    }
}


void GPhysicsWorld::DebugDraw(IGlacierDraw* pDraw, uint32_t mask ) const
{
    GPRORILER_FUN

    for( std::map<GGridPosition, GGridCell*>::const_iterator iter = m_Grids.begin(); iter != m_Grids.end(); ++iter )
    {
        iter->second->DebugDraw(pDraw, mask);
    }

    if (mask & GPDraw_Contact)
    {
        for (uint32_t i = 0; i < (uint32_t)m_BroadPhasePairs.size(); ++i)
        {
            const GBroadPhasePair& TestPair = m_BroadPhasePairs[i];
            GPhyscsUtils::DrawContact( TestPair.PairContact, pDraw, GColor::White(), this );
        }
    }

    for( int32_t i = 0; i < (int32_t)m_StaticLargeObj.size(); ++i )
    {
        DebugDrawObject( pDraw, m_StaticLargeObj[i], mask );
    }
}

void GPhysicsWorld::CollisionBroadPhase( )
{
    GPRORILER_FUN


    ClearContactPair();

    for (std::map<GGridPosition, GGridCell*>::const_iterator iterA = m_Grids.begin(); iterA != m_Grids.end(); ++iterA)
    {
        // ceil 
        for (int32_t i = 0; i < (int32_t)iterA->second->m_Objects.size(); ++i)
        {
            GCObject* pObjectA = iterA->second->m_Objects[i];

            const GAABB& BoxA = pObjectA->GetAABB();
            for (int32_t j = 0; j < i; ++j)
            {
                GCObject* pObjectB = iterA->second->m_Objects[j];
                const GAABB& BoxB = pObjectB->GetAABB();
                if (BoxA.Intersects(BoxB))
                {
                    AddContactPair(pObjectA, pObjectB);
                }
            }


            for ( int32_t nLargeObj = 0; nLargeObj < (int32_t)m_StaticLargeObj.size(); ++nLargeObj )
            {
                GCObject* pLargeObj = m_StaticLargeObj[nLargeObj];
                const GAABB& BoxB = pLargeObj->GetAABB();
                if (BoxA.Intersects(BoxB))
                {
                    AddContactPair(pObjectA, pLargeObj);
                }
            }
        }
        // other ceil

        for (int32_t i = 0; i < 13; ++i)
        {
            GGridPosition TPos = s_BroadPhaseNeighbourt[i] + iterA->first;

            std::map<GGridPosition, GGridCell*>::const_iterator iterB = m_Grids.find(TPos);

            if (iterB == m_Grids.end())
                continue;

            for (int32_t LoopA = 0; LoopA < (int32_t)iterA->second->m_Objects.size(); ++LoopA)
            {
                GCObject*       pObjectA    = iterA->second->m_Objects[LoopA];
                const GAABB&    BoxA        = pObjectA->GetAABB();


                for (int32_t LoopB = 0; LoopB < (int32_t)iterB->second->m_Objects.size(); ++LoopB)
                {
                    GCObject* pObjectB = iterB->second->m_Objects[LoopB];
                    const GAABB& BoxB = pObjectB->GetAABB();
                    if (BoxA.Intersects(BoxB))
                    {
                        AddContactPair(pObjectA, pObjectB);
                    }
                }
            }
        }
    }
}

void GPhysicsWorld::CollisionNarrowPhase( )
{
    GPRORILER_FUN
    for( int32_t i = 0; i < (int32_t)m_Objects.size(); ++i )
    {
        m_Objects[i]->ClearContactPair();
    }
        
    for( int32_t i = 0; i < (int32_t)m_BroadPhasePairs.size(); ++i )
    {
        GBroadPhasePair& TestPair = m_BroadPhasePairs[i];

        int32_t ShapeTypeA = (int32_t)TestPair.pObjectA->m_Shape.ShapType;
        int32_t ShapeTypeB = (int32_t)TestPair.pObjectB->m_Shape.ShapType;

        GCollisionAlgorithm* pAlgorithm = m_CollisionManager.GetAlgrithm( ShapeTypeA, ShapeTypeB );

        if( pAlgorithm != nullptr )
        {
            TestPair.PairContact.ClearPoint( );
            pAlgorithm->ProcessCollision( TestPair.pObjectA, TestPair.pObjectB, &TestPair.PairContact );

            if( TestPair.PairContact.GetPointCount() > 0 )
            {
                TestPair.pObjectA->AddContactPair(TestPair.PairId, i);
                TestPair.pObjectB->AddContactPair(TestPair.PairId, i);
            }
        }
    }
}

GVector3 GetWorldRelative( const GVector3& VPos, GCObject* pObjectA, GCObject* pObjectB )
{
    GVector3 V1 = GVector3::Zero();
    if (pObjectA->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRA = (GRigidBody*)pObjectA;
        V1 = pRA->GetWorldPosVelocity( VPos );
    }

    GVector3 V2 = GVector3::Zero();
    if (pObjectB->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRB = (GRigidBody*)pObjectB;
        V2 = pRB->GetWorldPosVelocity(VPos);
    }

    return V1 - V2;

}

void GPhysicsWorld::SolveContactConstraint( GBroadPhasePair& pPair )
{
    int32_t nPointCount = pPair.PairContact.GetPointCount();
    if( nPointCount > 0 )
    {
        bool bSwap = pPair.PairContact.PointOnSurface == pPair.pObjectA->GetId() ? false : true;

        f32 factorA = GMath::Makef32(0, 4, 10);
        f32 factorB = -factorA;

      

        for( int32_t nLoop = 0; nLoop < 100; nLoop ++ )
        {

            bool bSeparate = true;
            for (int32_t i = 0; i < nPointCount; ++i)
            {
                GManifoldPoint& TPoint = pPair.PairContact.m_Point[i];

                GVector3 VNormal = bSwap ? -TPoint.m_Normal : TPoint.m_Normal;

                if (pPair.pObjectA->GetCOType() == CO_Rigid_Body)
                {
                    GRigidBody* pRA = (GRigidBody*)pPair.pObjectA;

                    if (pRA->m_bDynamic)
                        pRA->AddImpulse_World(TPoint.m_PosWorld, VNormal * factorA);
                }

                if (pPair.pObjectB->GetCOType() == CO_Rigid_Body)
                {
                    GRigidBody* pRB = (GRigidBody*)pPair.pObjectB;

                    if (pRB->m_bDynamic)
                        pRB->AddImpulse_World(TPoint.m_PosWorld, VNormal * factorB);
                }

                GVector3 VWorldVelocity =   GetWorldRelative( TPoint.m_PosWorld, pPair.pObjectA, pPair.pObjectB);

                if( GVector3::DotProduct( VWorldVelocity, VNormal ) < GMath::Zero()  )
                {
                    bSeparate = false;
                }

            }

            if(bSeparate)
            {
                break;
            }
        }


    }




}


void GPhysicsWorld::SolveContactConstraint( )
{
    GPRORILER_FUN

    for (uint32_t i = 0; i < (uint32_t)m_BroadPhasePairs.size(); ++i)
    {
        GBroadPhasePair& TPair = m_BroadPhasePairs[i];
        SolveContactConstraint( TPair );
    }



}

void GPhysicsWorld::UpdateSceneGrid( )
{
    GPRORILER_FUN

    for (int32_t i = 0; i < (int32_t)m_Objects.size(); ++i)
    {
        GCObject* pObject = m_Objects[i];
        if( pObject->m_bNeedUpdate )
        {
            pObject->UpdateAABB();

            UpdateCollisionObject(pObject);  
            pObject->m_bNeedUpdate = false;
        }
    }

    for (std::map<GGridPosition, GGridCell*>::const_iterator iter = m_Grids.begin(); iter != m_Grids.end(); ++iter)
    {
        if( iter->second->m_Objects.size() == 0 )
        {
            delete iter->second;
            m_Grids.erase( iter++ );
        }
    }

    for (int32_t i = 0; i < (int32_t)m_StaticLargeObj.size(); ++i)
    {
        GCObject* pObject = m_StaticLargeObj[i];
        if (pObject->m_bNeedUpdate)
        {
            pObject->UpdateAABB();
        }
    }
}

void GPhysicsWorld::ClearContactPair()
{
    m_BroadPhasePairs.clear();
    m_BroadPhaseMap.clear();
}
void GPhysicsWorld::AddContactPair(GCObject* p1, GCObject* p2)
{
    GBroadPhasePair  TPair(p1, p2);

    m_BroadPhasePairs.push_back(TPair);

    m_BroadPhaseMap[TPair.PairId] = TPair;
}
