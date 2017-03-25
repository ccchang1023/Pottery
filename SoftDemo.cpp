/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///btSoftBody implementation by Nathanael Presson


#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include "../GimpactTestDemo/BunnyMesh.h"
#include "../GimpactTestDemo/TorusMesh.h"
#include <stdio.h>
#include "LinearMath/btConvexHull.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"   // In "Bullet/Extra/Serialize". Needs link to: BulletWorldImporter.lib

#include "DemoApplication.h"
#include "SoftDemo.h"
#include "GL_ShapeDrawer.h"
#include "GLDebugFont.h"
#include "GlutStuff.h"


//-----------------AR---------------------
#ifdef _WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#ifndef __APPLE__
#include <GL/gl.h>
#include <GL/glut.h>
#else
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#endif
#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
int             count = 0;
static void AR_cleanup(void);
int             thresh = 100;
int             marker_num;
    int             j, k;
	int             patt_width     = 80.0;
    double          patt_center[2] = {0.0, 0.0};
    double          patt_trans[3][4];
	int             patt_id;
	char           *patt_name      = "Data\\patt.hiro";
	int             mode = 1;
	int				contF=0;
	double			gl_para[16];

//-----------------------AR END------------------------


/*leap control relative*/
#include <iostream>
#include "Leap.h"
using namespace Leap;
/*leap control relative*/

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

static bool sDemoMode = false;

const int maxProxies = 32766;
const int maxOverlap = 65535;

static btVector3*	gGroundVertices=0;
static int*	gGroundIndices=0;
static btBvhTriangleMeshShape* trimeshShape =0;
static btRigidBody* staticBody = 0;
static float waveheight = 5.f;
ARUint8	*latest_dataPtr;  // non Null latest AR camera pointer

const float TRIANGLE_SIZE=8.f;
int		current_demo=20;
#define DEMO_MODE_TIMEOUT 1500.f //15 seconds for each demo

#ifdef _DEBUG
const int gNumObjects = 1;
#else
const int gNumObjects = 1;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cp
#endif

const int maxNumObjects = 32760;

#define CUBE_HALF_EXTENTS 1.5
#define EXTRA_HEIGHT -10.f



// -----------------defined in AR teapot material--------------
float   light_position[]  = {100.0,-200.0,200.0,0.0};
float   ambi[]            = {0.1, 0.1, 0.1, 0.1};
float   lightZeroColor[]  = {0.9, 0.9, 0.9, 0.1};
float   mat_flash[]       = {0.0, 0.0, 1.0, 1.0};
float   mat_flash_shiny[] = {50.0};
float   mat_ambient[]     = {0.0, 0.0, 1.0, 1.0};
static GLuint texName;
//-----------------------------------------------------------------

#ifdef USE_AMD_OPENCL
//....//
#endif //USE_AMD_OPENCL

//
void SoftDemo::createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos )
{
	btTransform trans;
	trans.setIdentity();

	for(int i=0; i<size; i++)
	{
		// This constructs a row, from left to right
		int rowSize = size - i;
		for(int j=0; j< rowSize; j++)
		{
			btVector3 pos;
			pos.setValue(
				-rowSize * halfCubeSize + halfCubeSize + j * 2.0f * halfCubeSize,
				halfCubeSize + i * halfCubeSize * 2.0f,
				zPos);

			trans.setOrigin(pos);
			btScalar mass = 1.f;

			btRigidBody* body = 0;
			body = localCreateRigidBody(mass,trans,boxShape);

		}
	}
}


////////////////////////////////////

extern int gNumManifold;
extern int gOverlappingPairs;

///for mouse picking
void pickingPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	SoftDemo* softDemo = (SoftDemo*)world->getWorldUserInfo();

	if(softDemo->m_drag)
	{
		const int				x=softDemo->m_lastmousepos[0];
		const int				y=softDemo->m_lastmousepos[1];
		const btVector3			rayFrom=softDemo->getCameraPosition();
		const btVector3			rayTo=softDemo->getRayTo(x,y);
		const btVector3			rayDir=(rayTo-rayFrom).normalized();
		const btVector3			N=(softDemo->getCameraTargetPosition()-softDemo->getCameraPosition()).normalized();
		const btScalar			O=btDot(softDemo->m_impact,N);
		const btScalar			den=btDot(N,rayDir);
		if((den*den)>0)
		{
			const btScalar			num=O-btDot(N,rayFrom);
			const btScalar			hit=num/den;
			if((hit>0)&&(hit<1500))
			{				
				softDemo->m_goal=rayFrom+rayDir*hit;
			}				
		}		
		btVector3				delta=softDemo->m_goal-softDemo->m_node->m_x;
		static const btScalar	maxdrag=10;
		if(delta.length2()>(maxdrag*maxdrag))
		{
			delta=delta.normalized()*maxdrag;
		}
		softDemo->m_node->m_v+=delta/timeStep;
	}

}



void SoftDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	glFlush();
	swapBuffers();
}


//
// ImplicitShape
//

//
struct	ImplicitSphere : btSoftBody::ImplicitFn
{
	btVector3	center;
	btScalar	sqradius;
	ImplicitSphere() {}
	ImplicitSphere(const btVector3& c,btScalar r) : center(c),sqradius(r*r) {}
	btScalar	Eval(const btVector3& x)
	{
		return((x-center).length2()-sqradius);
	}
};

//
// Tetra meshes
//

struct	TetraBunny
{
#include "bunny.inl"
};

struct	TetraCube
{
#include "cube.inl"
};


//
// Random
//

static inline btScalar	UnitRand()
{
	return(rand()/(btScalar)RAND_MAX);
}

static inline btScalar	SignedUnitRand()
{
	return(UnitRand()*2-1);
}

static inline btVector3	Vector3Rand()
{
	const btVector3	p=btVector3(SignedUnitRand(),SignedUnitRand(),SignedUnitRand());
	return(p.normalized());
}

//
// Rb rain
//
static void	Ctor_RbUpStack(SoftDemo* pdemo,int count)
{
	float				mass=10;

	btCompoundShape* cylinderCompound = new btCompoundShape;
	btCollisionShape* cylinderShape = new btCylinderShapeX(btVector3(4,1,1));
	btCollisionShape* boxShape = new btBoxShape(btVector3(4,1,1));
	btTransform localTransform;
	localTransform.setIdentity();
	cylinderCompound->addChildShape(localTransform,boxShape);
	btQuaternion orn(SIMD_HALF_PI,0,0);
	localTransform.setRotation(orn);
	//	localTransform.setOrigin(btVector3(1,1,1));
	cylinderCompound->addChildShape(localTransform,cylinderShape);


	btCollisionShape*	shape[]={cylinderCompound,
		new btBoxShape(btVector3(1,1,1)),
		new btSphereShape(1.5)
		
	};
	static const int	nshapes=sizeof(shape)/sizeof(shape[0]);
	for(int i=0;i<count;++i)
	{
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(0,2+6*i,0));
		pdemo->localCreateRigidBody(mass,startTransform,shape[i%nshapes]);
		//pdemo->localCreateRigidBody(mass,startTransform,shape[0]);
	}
}

//
// Big ball
//
static void	Ctor_BigBall(SoftDemo* pdemo,btScalar mass=10)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,13,0));
	pdemo->localCreateRigidBody(mass,startTransform,new btSphereShape(3));
}

//
// Big plate
//
static btRigidBody*	Ctor_BigPlate(SoftDemo* pdemo,btScalar mass=50,btScalar height=4)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(10,height,5));
	btRigidBody*		body=pdemo->localCreateRigidBody(mass,startTransform,new btBoxShape(btVector3(1,1,1)));
	body->setFriction(1);
	return(body);
}

//
// Linear stair
//
static void Ctor_LinearStair(SoftDemo* pdemo,const btVector3& org,const btVector3& sizes,btScalar angle,int count)
{
	btBoxShape*	shape=new btBoxShape(sizes);
	for(int i=0;i<count;++i)
	{
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(org+btVector3(sizes.x()*i*2,sizes.y()*i*2,0));
		btRigidBody* body=pdemo->localCreateRigidBody(0,startTransform,shape);
		body->setFriction(1);	
	}
}

//
// Softbox
//
static btSoftBody* Ctor_SoftBox(SoftDemo* pdemo,const btVector3& p,const btVector3& s)
{
	const btVector3	h=s*0.5;
	const btVector3	c[]={	p+h*btVector3(-1,-1,-1),
		p+h*btVector3(+1,-1,-1),
		p+h*btVector3(-1,+1,-1),
		p+h*btVector3(+1,+1,-1),
		p+h*btVector3(-1,-1,+1),
		p+h*btVector3(+1,-1,+1),
		p+h*btVector3(-1,+1,+1),
		p+h*btVector3(+1,+1,+1)};
	btSoftBody*		psb=btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo,c,8);
	psb->generateBendingConstraints(2);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	return(psb);

}

static void	Init_Aero(SoftDemo* pdemo)
{
	//TRACEDEMO
	const btScalar	s=2;
	const btScalar	h=10;
	const int		segments=6;
	const int		count=50;
	for(int i=0;i<count;++i)
	{
		btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(pdemo->m_softBodyWorldInfo,btVector3(-s,h,-s),
			btVector3(+s,h,-s),
			btVector3(-s,h,+s),
			btVector3(+s,h,+s),
			segments,segments,
			0,true);
		btSoftBody::Material*	pm=psb->appendMaterial();
		pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
		psb->generateBendingConstraints(2,pm);
		psb->m_cfg.kLF			=	0.004;
		psb->m_cfg.kDG			=	0.0003;
		psb->m_cfg.aeromodel	=	btSoftBody::eAeroModel::V_TwoSided;
		btTransform		trs;
		btQuaternion	rot;
		btVector3		ra=Vector3Rand()*0.1;
		btVector3		rp=Vector3Rand()*15+btVector3(0,20,80);
		rot.setEuler(SIMD_PI/8+ra.x(),-SIMD_PI/7+ra.y(),ra.z());
		trs.setIdentity();
		trs.setOrigin(rp);
		trs.setRotation(rot);
		psb->transform(trs);
		psb->setTotalMass(0.1);
		psb->addForce(btVector3(0,2,0),0);
		pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	}
	pdemo->m_autocam=true;
}

static void	Init_Aero2(SoftDemo* pdemo)
{
	//TRACEDEMO
	const btScalar	s=5;
	const int		segments=10;
	const int		count=5;
	btVector3 pos(-s*segments, 0, 0);
	btScalar gap = 0.5;

	for(int i=0;i<count;++i)
	{
		btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(	pdemo->m_softBodyWorldInfo,btVector3(-s,0,-s*3),
			btVector3(+s,0,-s*3),
			btVector3(-s,0,+s),
			btVector3(+s,0,+s),
			segments,segments*3,
			1+2,true);
		
		psb->getCollisionShape()->setMargin(0.5);
		btSoftBody::Material* pm=psb->appendMaterial();
		pm->m_kLST		=	0.0004;
		pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
		psb->generateBendingConstraints(2,pm);
		
		psb->m_cfg.kLF			=	0.05;
		psb->m_cfg.kDG			=	0.01;

		//psb->m_cfg.kLF			=	0.004;
		//psb->m_cfg.kDG			=	0.0003;

		psb->m_cfg.piterations = 2;
		psb->m_cfg.aeromodel	=	btSoftBody::eAeroModel::V_TwoSidedLiftDrag;

		
		psb->setWindVelocity(btVector3(4, -12.0, -25.0));

		btTransform		trs;
		btQuaternion	rot;
		pos += btVector3(s*2 + gap, 0, 0);
		rot.setRotation(btVector3(1, 0, 0), btScalar(SIMD_PI/2));
		trs.setIdentity();
		trs.setOrigin(pos);
		trs.setRotation(rot);
		psb->transform(trs);
		psb->setTotalMass(2.0);
		
		pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	}

	pdemo->m_autocam=true;
}

//
// Friction
//
static void	Init_Friction(SoftDemo* pdemo)
{
	//TRACEDEMO
	const btScalar	bs=2;
	const btScalar	ts=bs+bs/4;
	for(int i=0,ni=20;i<ni;++i)
	{
		const btVector3	p(-ni*ts/2+i*ts,-10+bs,40);
		btSoftBody*		psb=Ctor_SoftBox(pdemo,p,btVector3(bs,bs,bs));
		psb->m_cfg.kDF	=	0.1 * ((i+1)/(btScalar)ni);
		psb->addVelocity(btVector3(0,0,-10));
	}
}

//
// Pressure
//
static void	Init_Pressure(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateEllipsoid(pdemo->m_softBodyWorldInfo,btVector3(0,180,0),
		btVector3(3,5,3)*26,
		750);
	psb->m_materials[0]->m_kLST	=	0.2; // Linear stiffness coefficient [0,1]
	//psb->m_materials[0]->m_kAST = 1;			// Area/Angular stiffness coefficient [0,1]
	//psb->m_materials[0]->m_kVST = 1;			// Volume stiffness coefficient [0,1]
	psb->m_cfg.kDF				=	0.4;	//Dynamic friction coefficient [0,1]
	psb->m_cfg.kDP				=	0.5;  // Damping coefficient [0,1]
	//psb->m_cfg.kLF = 10000;  //Lift coefficient [0,+inf]
	//psb->m_cfg.kVC = 0.1;   // Volume conversation coefficient [0,+inf]	
	//psb->m_cfg.kKHR = 1;		// Kinetic contacts hardness [0,1]
	//psb->m_cfg.kCHR = 1;	 // Rigid contacts hardness [0,1]
	//psb->m_cfg.kSHR = 1;			// Soft contacts hardness [0,1]
	psb->m_cfg.kPR				=	50; //Pressure coefficient [-inf,+inf]
	psb->setTotalMass(30,true);	
	//psb->setRestitution(1231);

	//create the bottom plate
	/*btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,25,10));
	btRigidBody*		plate=pdemo->localCreateRigidBody(50,startTransform,new btCylinderShape(btVector3(100,4,100)));
	plate->setFriction(0);*/


	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,25,10));
	pdemo->getSoftDynamicsWorld()->setGravity(btVector3(0,0,0));
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	//Ctor_LinearStair(pdemo,btVector3(0,0,0),btVector3(2,1,5),0,10);
	pdemo->m_autocam=true;

}

static struct MotorControl : btSoftBody::AJoint::IControl
{
	MotorControl()
	{
		goal=0;
		maxtorque=0;
	}
	btScalar	Speed(btSoftBody::AJoint*,btScalar current)
	{
		return(current+btMin(maxtorque,btMax(-maxtorque,goal-current)));
	}
	btScalar	goal;
	btScalar	maxtorque;
}	motorcontrol;

//
struct SteerControl : btSoftBody::AJoint::IControl
{
	SteerControl(btScalar s)
	{
		angle=0;
		sign=s;
	}
	void		Prepare(btSoftBody::AJoint* joint)
	{
		joint->m_refs[0][0]=btCos(angle*sign);
		joint->m_refs[0][2]=btSin(angle*sign);
	}
	btScalar	Speed(btSoftBody::AJoint* joint,btScalar current)
	{
		return(motorcontrol.Speed(joint,current));
	}
	btScalar	angle;
	btScalar	sign;
};

static SteerControl	steercontrol_f(+1);
static SteerControl	steercontrol_r(-1);

//
// TetraCube
//
static void	Init_TetraCube(SoftDemo* pdemo)
{
	btSoftBody* psb=btSoftBodyHelpers::CreateFromTetGenData(pdemo->m_softBodyWorldInfo,
															TetraCube::getElements(),
															0,
															TetraCube::getNodes(),
															false,true,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	psb->scale(btVector3(4,4,4));
	psb->translate(btVector3(0,5,0));
	psb->setVolumeMass(300);
	
	
	
	///fix one vertex
	//psb->setMass(0,0);
	//psb->setMass(10,0);
	//psb->setMass(20,0);
	psb->m_cfg.piterations=1;
	//psb->generateClusters(128);
	psb->generateClusters(16);
	//psb->getCollisionShape()->setMewrargin(0.5);

	psb->getCollisionShape()->setMargin(0.01);
	psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+	btSoftBody::fCollision::CL_RS
		//+ btSoftBody::fCollision::CL_SELF
		;
	psb->m_materials[0]->m_kLST=0.8;
	pdemo->m_cutting=false;	
	psb->setRestitution(0.01);
}





	/* Init		*/ 
	void (*demofncs[])(SoftDemo*)=
	{
		//Init_Cloth,
		Init_Pressure,
		//Init_Volume,
		//Init_Ropes,
		//Init_RopeAttach,
		//Init_ClothAttach,
		//Init_Sticks,
		//Init_CapsuleCollision,
		//Init_Collide,
		//Init_Collide2,
		//Init_Collide3,
		//Init_Impact,
		Init_Aero,
		Init_Aero2,
		Init_Friction,			
		//Init_Torus,
		//Init_TorusMatch,
		//Init_Bunny,
		//Init_BunnyMatch,
		//Init_Cutting1,
		//Init_ClusterDeform,
		//Init_ClusterCollide1,
		//Init_ClusterCollide2,
		//Init_ClusterSocket,
		//Init_ClusterHinge,
		//Init_ClusterCombine,
		//Init_ClusterCar,
		//Init_ClusterRobot,
		//Init_ClusterStackSoft,
		//Init_ClusterStackMixed,
		Init_TetraCube,
		//Init_TetraBunny,
	};

void	SoftDemo::clientResetScene()
{
	m_azi = 0;
	m_cameraDistance = 280.f;
	m_cameraTargetPosition.setValue(0,0,0);

	DemoApplication::clientResetScene();
	/* Clean up	*/ 
	for(int i=m_dynamicsWorld->getNumCollisionObjects()-1;i>=0;i--)
	{
		btCollisionObject*	obj=m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}
		while(m_dynamicsWorld->getNumConstraints())
		{
			btTypedConstraint*	pc=m_dynamicsWorld->getConstraint(0);
			m_dynamicsWorld->removeConstraint(pc);
			delete pc;
		}
		btSoftBody* softBody = btSoftBody::upcast(obj);
		if (softBody)
		{
			getSoftDynamicsWorld()->removeSoftBody(softBody);
		} else
		{
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body)
				m_dynamicsWorld->removeRigidBody(body);
			else
				m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}


	//create ground object
	/*btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-12,0));

	btCollisionObject* newOb = new btCollisionObject();
	newOb->setWorldTransform(tr);
	newOb->setInterpolationWorldTransform( tr);*/
	int lastDemo = (sizeof(demofncs)/sizeof(demofncs[0]))-1;

	if (current_demo<0)
		current_demo = lastDemo;
	if (current_demo > lastDemo)
		current_demo =0;
		

	/*if (current_demo>19)
	{
		newOb->setCollisionShape(m_collisionShapes[0]);
	} else
	{
		newOb->setCollisionShape(m_collisionShapes[1]);
	}

	m_dynamicsWorld->addCollisionObject(newOb);*/

	m_softBodyWorldInfo.m_sparsesdf.Reset();

	motorcontrol.goal = 0;
	motorcontrol.maxtorque = 0;

	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
	m_softBodyWorldInfo.water_density	=	0;
	m_softBodyWorldInfo.water_offset		=	0;
	m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
	m_softBodyWorldInfo.m_gravity.setValue(0,0,0);


	m_autocam						=	false;
	m_raycast						=	false;
	m_cutting						=	false;
	m_results.fraction				=	1.f;
	
	spin_switch = 0;
	spin_rate = 5;

	//demofncs[current_demo](this);
	demofncs[0](this);
}


void SoftDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);

	float ms = getDeltaTimeMicroseconds();
	float dt = ms / 1000000.f;//1.0/60.;	

	if (m_dynamicsWorld)
	{
		
		if (sDemoMode)
		{
			static float demoCounter = DEMO_MODE_TIMEOUT;
			demoCounter-= dt;
			if (demoCounter<0)
			{
				
				demoCounter=DEMO_MODE_TIMEOUT;
				current_demo++;
				current_demo=current_demo%(sizeof(demofncs)/sizeof(demofncs[0]));
				clientResetScene();
			}
		}
		

//#define FIXED_STEP
#ifdef FIXED_STEP
		m_dynamicsWorld->stepSimulation(dt=1.0f/60.f,0);

#else
		//during idle mode, just run 1 simulation step maximum, otherwise 4 at max
	//	int maxSimSubSteps = m_idle ? 1 : 4;
		//if (m_idle)
		//	dt = 1.0/420.f;

		int numSimSteps;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt);
		//numSimSteps = m_dynamicsWorld->stepSimulation(dt,10,1./240.f);

#ifdef VERBOSE_TIMESTEPPING_CONSOLEOUTPUT
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_TIMESTEPPING_CONSOLEOUTPUT

#endif		

#ifdef USE_AMD_OPENCL
		if (g_openCLSIMDSolver)
			g_openCLSIMDSolver->copyBackToSoftBodies();
#endif //USE_AMD_OPENCL

		if(m_drag)
		{
			m_node->m_v*=0;
		}

		m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();

		//optional but useful: debug drawing

	}

#ifdef USE_QUICKPROF 
	btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 

	renderme(); 

	//render the graphics objects, with center of mass shift

	updateCamera();



#ifdef USE_QUICKPROF 
	btProfiler::endBlock("render"); 
#endif 
	glFlush();
	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	printf("num manifolds: %i\n",gNumManifold);
	printf("num gOverlappingPairs: %i\n",gOverlappingPairs);
	
#endif //PRINT_CONTACT_STATISTICS


	swapBuffers();

}

void	SoftDemo::renderme()
{
	//AR parameters
	ARUint8         *dataPtr;  // AR camera pointer
	ARMarkerInfo    *marker_info;
    int             marker_num;
    int             j, k;
	double    gl_para[16];
	//AR part end	
	//-------------------AR draw background------
	//glColor3f(0.5,0.5,0.5);
	if( (dataPtr = (ARUint8 *)arVideoGetImage()) == NULL ) {
		if(!latest_dataPtr)
			return;
		dataPtr = latest_dataPtr;
		//arUtilSleep(2);
    }
	else{
		latest_dataPtr = dataPtr;
		if( count == 0 ) arUtilTimerReset();
		count++;
	}
	argDrawMode2D();
	argDispImage( dataPtr, 0,0 );
//detect marker
	if( arDetectMarker(dataPtr, thresh, &marker_info, &marker_num) < 0 ) {
        arVideoCapStop();
        arVideoClose();
        argCleanup();
        exit(0);
    }
	//get new frame
	arVideoCapNext();
	//match maker
	k = -1;
    for( j = 0; j < marker_num; j++ ) {
        if( patt_id == marker_info[j].id ) {
            if( k == -1 ) k = j;
            else if( marker_info[k].cf < marker_info[j].cf ) k = j;
        }
    }
    /*if( k == -1 ) {
		argSwapBuffers();
        return;
    }*/
	//get patt_trans 
	//if(position_switch == 0)
	if(k != -1)
	arGetTransMat(&marker_info[k], patt_center, patt_width, patt_trans);

	argDrawMode3D();
    argDraw3dCamera( 0, 0 );
    //glClearDepth( 1.0 );
    //glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_LEQUAL);
	argConvGlpara(patt_trans, gl_para);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd( gl_para );	
	glRotatef(90.0, 1.0, 0.0, 0.0);
	glScalef(softbody_scale,softbody_scale,softbody_scale);
	//glScalef(0.3,0.3,0.3);
	//glTranslatef(0,-250,0);






	//ARUint8         *dataPtr;  // AR camera pointer
	//ARMarkerInfo    *marker_info;  
	btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();
	//glDisable(GL_TEXTURE_2D);
	//glDisable(GL_LIGHTING);
	m_dynamicsWorld->debugDrawWorld();
	GLfloat light_specular[]={1.0,1.0,1.0,1.0};
	GLfloat mat_specular[]={0.5,0.0,0.0,0.0};	
   //---------------------------------------

	//int debugMode = m_dynamicsWorld->getDebugDrawer()? m_dynamicsWorld->getDebugDrawer()->getDebugMode() : -1;
	btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	//btIDebugDraw*	sdraw = softWorld ->getDebugDrawer();


	for (  int i=0;i<softWorld->getSoftBodyArray().size();i++)
	{
		btSoftBody*	psb=(btSoftBody*)softWorld->getSoftBodyArray()[i];
		if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
		{
			btSoftBodyHelpers::DrawFrame(psb,softWorld->getDebugDrawer());
			btSoftBodyHelpers::Draw(psb,softWorld->getDebugDrawer(),softWorld->getDrawFlags());

		}
	}

    /* Bodies		*/ 
	btVector3	ps(0,0,0);
	int			nps=0;

	btSoftBodyArray&	sbs=getSoftDynamicsWorld()->getSoftBodyArray();
	for(int ib=0;ib<sbs.size();++ib)
	{
		btSoftBody*	psb=sbs[ib];
		nps+=psb->m_nodes.size();
		for(int i=0;i<psb->m_nodes.size();++i)
		{
			ps+=psb->m_nodes[i].m_x;
		}		
	}
	ps/=nps;
	if(m_autocam)
		m_cameraTargetPosition+=(ps-m_cameraTargetPosition)*0.05;
	/* Anm			*/ 
	if(!isIdle())
		m_animtime=m_clock.getTimeMilliseconds()/1000.f;

	/* Ray cast		*/ 
	/*if(m_raycast)
	{		
		// Prepare rays 
		const int		res=64;
		const btScalar	fres=res-1;
		const btScalar	size=8;
		const btScalar	dist=10;
		btTransform		trs;
		trs.setOrigin(ps);
		btScalar rayLength = 1000.f;

		const btScalar	angle=m_animtime*0.2;
		trs.setRotation(btQuaternion(angle,SIMD_PI/4,0));
		btVector3	dir=trs.getBasis()*btVector3(0,-1,0);
		trs.setOrigin(ps-dir*dist);
		btAlignedObjectArray<btVector3>	origins;
		btAlignedObjectArray<btScalar>	fractions;
		origins.resize(res*res);
		fractions.resize(res*res,1.f);
		for(int y=0;y<res;++y)
		{
			for(int x=0;x<res;++x)
			{
				const int	idx=y*res+x;
				origins[idx]=trs*btVector3(-size+size*2*x/fres,dist,-size+size*2*y/fres);
			}
		}
		// Cast rays
		{
			m_clock.reset();
			if (sbs.size())
			{
				btVector3*		org=&origins[0];
				btScalar*				fraction=&fractions[0];
				btSoftBody**			psbs=&sbs[0];
				btSoftBody::sRayCast	results;
				for(int i=0,ni=origins.size(),nb=sbs.size();i<ni;++i)
				{
					for(int ib=0;ib<nb;++ib)
					{
						btVector3 rayFrom = *org;
						btVector3 rayTo = rayFrom+dir*rayLength;
						if(psbs[ib]->rayTest(rayFrom,rayTo,results))
						{
							*fraction=results.fraction;
						}
					}
					++org;++fraction;
				}
				long	ms=btMax<long>(m_clock.getTimeMilliseconds(),1);
				long	rayperseconds=(1000*(origins.size()*sbs.size()))/ms;
				printf("%d ms (%d rays/s)\r\n",int(ms),int(rayperseconds));
			}
		}
		// Draw rays
		const btVector3	c[]={	origins[0],
			origins[res-1],
			origins[res*(res-1)],
			origins[res*(res-1)+res-1]};
		idraw->drawLine(c[0],c[1],btVector3(0,0,0));
		idraw->drawLine(c[1],c[3],btVector3(0,0,0));
		idraw->drawLine(c[3],c[2],btVector3(0,0,0));
		idraw->drawLine(c[2],c[0],btVector3(0,0,0));
		for(int i=0,ni=origins.size();i<ni;++i)
		{
			const btScalar		fraction=fractions[i];
			const btVector3&	org=origins[i];
			if(fraction<1.f)
			{
				idraw->drawLine(org,org+dir*rayLength*fraction,btVector3(1,0,0));
			}
			else
			{
				idraw->drawLine(org,org-dir*rayLength*0.1,btVector3(0,0,0));
			}
		}
#undef RES
	}*/
	// Water level
	/*static const btVector3	axis[]={btVector3(1,0,0),
		btVector3(0,1,0),
		btVector3(0,0,1)};
	if(m_softBodyWorldInfo.water_density>0)
	{
		const btVector3	c=	btVector3((btScalar)0.25,(btScalar)0.25,1);
		const btScalar	a=	(btScalar)0.5;
		const btVector3	n=	m_softBodyWorldInfo.water_normal;
		const btVector3	o=	-n*m_softBodyWorldInfo.water_offset;
		const btVector3	x=	btCross(n,axis[n.minAxis()]).normalized();
		const btVector3	y=	btCross(x,n).normalized();
		const btScalar	s=	25;
		idraw->drawTriangle(o-x*s-y*s,o+x*s-y*s,o+x*s+y*s,c,a);
		idraw->drawTriangle(o-x*s-y*s,o+x*s+y*s,o-x*s+y*s,c,a);
	}*/
	//

	/*int lineWidth=280;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	if((getDebugMode() & btIDebugDraw::DBG_NoHelpText)==0)
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		
		glRasterPos3f(xStart, yStart, 0);
		if (sDemoMode)
		{		
			sprintf(buf,"d to toggle demo mode (on)");
		} else
		{
			sprintf(buf,"d to toggle demo mode (off)");
		}
		GLDebugDrawString(xStart,20,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"] for next demo (%d)",current_demo);
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"c to visualize clusters");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"; to toggle camera mode");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
        sprintf(buf,"n,m,l,k for power and steering");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);


		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}*/

	//DemoApplication::stepLeft();
	if(spin_switch){
		btSoftBody*	psb=sbs[0];
		psb->rotate(btQuaternion(btVector3(0,1,0),btRadians(spin_rate)));
	}
	
	int	xOffset = 10;
	int yStart = 20;
	int yIncr = 20;
	/*if ((m_debugMode & btIDebugDraw::DBG_NoHelpText)==0){
		setOrthographicProjection();
		showProfileInfo(xOffset,yStart,yIncr);
		resetPerspectiveProjection();
	}*/
	DemoApplication::renderme();
	

}

void	SoftDemo::setDrawClusters(bool drawClusters)
{
	if (drawClusters)
	{
		getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags()|fDrawFlags::Clusters);
	} else
	{
		getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags()& (~fDrawFlags::Clusters));
	}
}


void	SoftDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch(key)
	{
	case    's':	spin_switch = (spin_switch)?false:true; forceInsert(detection_switch,spin_switch,spin_rate,force,softbody_scale); break; // open/close the switch
	case    'w':	position_switch = (position_switch)?false:true; break; // open/close the switch
	case	'r':	clientResetScene(); detection_switch=false; forceInsert(detection_switch,spin_switch,spin_rate,force,softbody_scale); break;  //reset the body
	case    'e':    detection_switch = (detection_switch)?false:true; forceInsert(detection_switch,spin_switch,spin_rate,force,softbody_scale); break;
	case	'x':	force+=30; forceInsert(detection_switch,spin_switch,spin_rate,force,softbody_scale); break;
	case	'z':	force-=30; forceInsert(detection_switch,spin_switch,spin_rate,force,softbody_scale); break;
	case    'd':	sDemoMode = !sDemoMode; break;
	case	'n':	motorcontrol.maxtorque=10;motorcontrol.goal+=1;break;
	case	'm':	motorcontrol.maxtorque=10;motorcontrol.goal-=1;break;
	case	'l':	steercontrol_f.angle+=0.1;steercontrol_r.angle+=0.1;break;
	case	'k':	steercontrol_f.angle-=0.1;steercontrol_r.angle-=0.1;break;
	/*case	']':	++current_demo;clientResetScene();break;
	case	'[':	--current_demo;clientResetScene();break;*/
	case	']':	spin_rate+=3; forceInsert(detection_switch,spin_switch,spin_rate,force,softbody_scale); break;
	case	'[':	spin_rate-=3; forceInsert(detection_switch,spin_switch,spin_rate,force,softbody_scale); break;
	case	'-':	softbody_scale+=0.01f; forceInsert(detection_switch,spin_switch,spin_rate,force,softbody_scale); break;
	case	'=':    softbody_scale-=0.01f; forceInsert(detection_switch,spin_switch,spin_rate,force,softbody_scale); break;
	case	'p':	spin_rate = 23; forceInsert(detection_switch,spin_switch,spin_rate,force,softbody_scale); break; // steady spin
	case	',':	m_raycast=!m_raycast;break;
	case	';':	m_autocam=!m_autocam;break;
	case	'c':	getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags()^fDrawFlags::Clusters);break;
	case	'`':
		{
			btSoftBodyArray&	sbs=getSoftDynamicsWorld()->getSoftBodyArray();
			for(int ib=0;ib<sbs.size();++ib)
			{
				btSoftBody*	psb=sbs[ib];
				psb->staticSolve(128);
			}
		}
		break;
	default:		DemoApplication::keyboardCallback(key,x,y);
	}
}


//
void	SoftDemo::mouseMotionFunc(int x,int y)
{
	if(m_node&&(m_results.fraction<1.f))
	{
		if(!m_drag)
		{
#define SQ(_x_) (_x_)*(_x_)
			if((SQ(x-m_lastmousepos[0])+SQ(y-m_lastmousepos[1]))>6)
			{
				m_drag=true;
			}
#undef SQ
		}
		if(m_drag)
		{
			m_lastmousepos[0]	=	x;
			m_lastmousepos[1]	=	y;		
		}
	}
	else
	{
		DemoApplication::mouseMotionFunc(x,y);
	}
}

//
void	SoftDemo::mouseFunc(int button, int state, int x, int y)
{
	if(button==0)
	{
		switch(state)
		{
			case	0:
			{
				std::cout<<"mouseclick callback x is "<< x << " and y is "<< y << std::endl;
				m_results.fraction=1.f;
				DemoApplication::mouseFunc(button,state,x,y);
				if(!m_pickConstraint)
				{
					const btVector3			rayFrom=m_cameraPosition;
					const btVector3			rayTo=getRayTo(x,y);
					const btVector3			rayDir=(rayTo-rayFrom).normalized();
					btSoftBodyArray&		sbs=getSoftDynamicsWorld()->getSoftBodyArray();
					for(int ib=0;ib<sbs.size();++ib)
					{
						btSoftBody*				psb=sbs[ib];
						btSoftBody::sRayCast	res;
						if(psb->rayTest(rayFrom,rayTo,res))
						{
							m_results=res;
						}
					}
					if(m_results.fraction<1.f)
					{				
						m_impact			=	rayFrom+(rayTo-rayFrom)*m_results.fraction;
						m_drag				=	m_cutting ? false : true;
						m_lastmousepos[0]	=	x;
						m_lastmousepos[1]	=	y;
						m_node				=	0;
						switch(m_results.feature)
						{
						case btSoftBody::eFeature::Tetra:
							{
								btSoftBody::Tetra&	tet=m_results.body->m_tetras[m_results.index];
								m_node=tet.m_n[0];
								for(int i=1;i<4;++i)
								{
									if(	(m_node->m_x-m_impact).length2()>
										(tet.m_n[i]->m_x-m_impact).length2())
									{
										m_node=tet.m_n[i];
									}
								}
								break;
							}
						case	btSoftBody::eFeature::Face:
							{
								btSoftBody::Face&	f=m_results.body->m_faces[m_results.index];
								m_node=f.m_n[0];
								for(int i=1;i<3;++i)
								{
									if(	(m_node->m_x-m_impact).length2()>
										(f.m_n[i]->m_x-m_impact).length2())
									{
										m_node=f.m_n[i];
									}
								}
							}
							break;
						}
						if(m_node) m_goal=m_node->m_x;
						return;
					}
				}
			}
			break;
		case	1:
			if((!m_drag)&&m_cutting&&(m_results.fraction<1.f))
			{
				ImplicitSphere	isphere(m_impact,1);
				printf("Mass before: %f\r\n",m_results.body->getTotalMass());
				m_results.body->refine(&isphere,0.0001,true);
				printf("Mass after: %f\r\n",m_results.body->getTotalMass());
			}
			m_results.fraction=1.f;
			m_drag=false;
			DemoApplication::mouseFunc(button,state,x,y);
			break;
		}
	}
	else
	{
		DemoApplication::mouseFunc(button,state,x,y);
	}
}


void	SoftDemo::leapCameraControl(float x)
{
	//m_ele += x;
	m_azi += 200*x;

}

void SoftDemo::leapControl(SoftDemo* pdemo,Leap::Vector dir,Leap::Vector pos,float spm,float width,bool is_finger)
{
	btSoftBodyArray &psb = pdemo->getSoftDynamicsWorld()->getSoftBodyArray();	
	btScalar nodes_num = psb[0]->m_nodes.size();
	btVector3 framePos = btVector3(pos.x,pos.y,pos.z);
	btVector3 frameDir = (is_finger)?btVector3(dir.x,dir.y,dir.z):btVector3(-dir.x,dir.y,-dir.z);
	btScalar dis;
	int contact_area = (is_finger)?25:33;   // not finger = palm;
	for(int i =0 ;i<nodes_num;i++){
		dis = psb[0]->m_nodes[i].m_x.distance(framePos);
		if(dis<=contact_area){
			psb[0]->addForce(force*frameDir,i); //spm: speed magnitude
		}
	}
}

void	SoftDemo::initPhysics()
{
	///create concave ground mesh
	m_azi = 0;
	//reset and disable motorcontrol at the start
	motorcontrol.goal = 0;
	motorcontrol.maxtorque = 0;

	btCollisionShape* groundShape = 0;
	{
		int i;
		int j;

		const int NUM_VERTS_X = 30;
		const int NUM_VERTS_Y = 30;
		const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
		const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

		gGroundVertices = new btVector3[totalVerts];
		gGroundIndices = new int[totalTriangles*3];

		btScalar offset(-50);

		for ( i=0;i<NUM_VERTS_X;i++)
		{
			for (j=0;j<NUM_VERTS_Y;j++)
			{
				gGroundVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
					//0.f,
					waveheight*sinf((float)i)*cosf((float)j+offset),
					(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
			}
		}

		int vertStride = sizeof(btVector3);
		int indexStride = 3*sizeof(int);

		int index=0;
		for ( i=0;i<NUM_VERTS_X-1;i++)
		{
			for (int j=0;j<NUM_VERTS_Y-1;j++)
			{
				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = j*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i;
			}
		}

		btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
			gGroundIndices,
			indexStride,
			totalVerts,(btScalar*) &gGroundVertices[0].x(),vertStride);

		bool useQuantizedAabbCompression = true;

		groundShape = new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression);
		groundShape->setMargin(0.5);
	}

	m_collisionShapes.push_back(groundShape);

	btCollisionShape* groundBox = new btBoxShape (btVector3(100,CUBE_HALF_EXTENTS,100));
	m_collisionShapes.push_back(groundBox);

	btCompoundShape* cylinderCompound = new btCompoundShape;
	btCollisionShape* cylinderShape = new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
	btTransform localTransform;
	localTransform.setIdentity();
	cylinderCompound->addChildShape(localTransform,cylinderShape);
	btQuaternion orn(btVector3(0,1,0),SIMD_PI);
	localTransform.setRotation(orn);
	cylinderCompound->addChildShape(localTransform,cylinderShape);

	m_collisionShapes.push_back(cylinderCompound);


	m_dispatcher=0;

	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();


	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	////////////////////////////
	///Register softbody versus softbody collision algorithm


	///Register softbody versus rigidbody collision algorithm


	////////////////////////////

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;

	btSoftBodySolver* softBodySolver = 0;
#ifdef USE_AMD_OPENCL

	static bool once = true;
	if (once)
	{
		once=false;
		initCL(0,0);
	}

	if( g_openCLSIMDSolver  )
		delete g_openCLSIMDSolver;
	if( g_softBodyOutput )
		delete g_softBodyOutput;

	if (1)
	{
		g_openCLSIMDSolver = new btOpenCLSoftBodySolverSIMDAware( g_cqCommandQue, g_cxMainContext);
	//	g_openCLSIMDSolver = new btOpenCLSoftBodySolver( g_cqCommandQue, g_cxMainContext);
		g_openCLSIMDSolver->setCLFunctions(new CachingCLFunctions(g_cqCommandQue, g_cxMainContext));
	}	



	softBodySolver = g_openCLSIMDSolver;
	g_softBodyOutput = new btSoftBodySolverOutputCLtoCPU;
#endif //USE_AMD_OPENCL

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration,softBodySolver);
	m_dynamicsWorld = world;
	m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback,this,true);


	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);

	//	clientResetScene();

	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	clientResetScene();

	// ar initialize
	if( (patt_id=arLoadPatt(patt_name)) < 0 ) {
        printf("pattern load error !!\n");
        exit(0);
    }
	/**/
}


void	SoftDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;



	delete m_collisionConfiguration;


}

static void AR_cleanup(void)
{
    arVideoCapStop();
    arVideoClose();
    argCleanup();
}

unsigned char *LoadBitmapFile(char *fileName, BITMAPINFO *bitmapInfo)
{
   FILE            *fp;
   BITMAPFILEHEADER   bitmapFileHeader;   // Bitmap file header
   unsigned char       *bitmapImage;      // Bitmap image data
   unsigned int      lInfoSize;         // Size of information
   unsigned int      lBitSize;         // Size of bitmap   
   unsigned char change;
    int pixel;
    int p=0;       
   fp = fopen(fileName, "rb");
   fread(&bitmapFileHeader, sizeof(BITMAPFILEHEADER), 1, fp);         //讀取 bitmap header   
   lInfoSize = bitmapFileHeader.bfOffBits - sizeof(BITMAPFILEHEADER);   //Info的size
   fread(bitmapInfo, lInfoSize, 1, fp);   
   lBitSize = bitmapInfo->bmiHeader.biSizeImage;                  //配置記憶體
   bitmapImage = new BYTE[lBitSize];
   fread(bitmapImage, 1, lBitSize, fp);                        //讀取影像檔   
   fclose(fp);   
   //此時傳回bitmapImage的話，顏色會是BGR順序，下面迴圈會改順序為RGB
   pixel = (bitmapInfo->bmiHeader.biWidth)*(bitmapInfo->bmiHeader.biHeight);
   for( int i=0 ; i<pixel ; i++, p+=3 )
   {
      //交換bitmapImage[p]和bitmapImage[p+2]的值
      change = bitmapImage[p];
      bitmapImage[p] = bitmapImage[p+2];
      bitmapImage[p+2]  = change;
   }   
   return bitmapImage;
} 



