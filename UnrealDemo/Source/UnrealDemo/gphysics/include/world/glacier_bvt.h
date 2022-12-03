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
#include "glacier_aabb.h"
#include "glacier_transform_qt.h"
#include "glacier_collision_shape.h"
#include <set>
#include <vector>


struct GBVNode
{
	GAABB		Box;
	GBVNode*	Parent;
	GBVNode*	Childs[2];
};


class GBVT
{
public:

	//void Update




	GBVNode* TreeRoot;
};