/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

// Convex demo.
// Serves as a test for the convex geometry.
// By Bram Stolk.

#include <assert.h>
#ifdef HAVE_UNISTD_H
#	include <unistd.h>
#endif
#if defined(linux)
#	include <fenv.h>
#endif
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#include "halton235_geom.h"

#ifdef dDOUBLE
#	define dsDrawConvex dsDrawConvexD
#	define dsDrawLine dsDrawLineD
#endif


#ifdef _MSC_VER
#	pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


// Height at which we drop the composite block.
const dReal H=4.20;

static dWorldID world;
static dSpaceID space;

static dBodyID mbody;

static dBodyID hbody[ halton_numc ];
static dGeomID hgeom[ halton_numc ];

static dJointGroupID contactgroup;

static bool drawpos=false;
static bool solidkernel=false;


// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	assert(o1);
	assert(o2);
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
	{
		// colliding a space with something
		dSpaceCollide2(o1,o2,data,&nearCallback);
		// Note we do not want to test intersections within a space,
		// only between spaces.
		return;
	}

	const int N = 32;
	dContact contact[N];
	int n = dCollide (o1,o2,N,&(contact[0].geom),sizeof(dContact));
	if ( n==32 )
		fprintf( stdout, "WARNING! collision exceeded 32 contacts.\n"  );
	if (n > 0) 
	{
		for (int i=0; i<n; i++) 
		{
			contact[i].surface.slip1 = 0.7;
			contact[i].surface.slip2 = 0.7;
			contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
			contact[i].surface.mu = 500.0; // was: dInfinity
			contact[i].surface.soft_erp = 0.50;
			contact[i].surface.soft_cfm = 0.03;
			dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
			dJointAttach
			(
				c,
				dGeomGetBody(contact[i].geom.g1),
				dGeomGetBody(contact[i].geom.g2)
			);
		}
	}
}


// start simulation - set viewpoint

static void start()
{
	dAllocateODEDataForThread(dAllocateMaskAll);
	static float xyz[3] = {-8,0,5};
	static float hpr[3] = {0.0f,-29.5000f,0.0000f};
	dsSetViewpoint (xyz,hpr);
	fprintf(stderr,"Press SPACE to reset the simulation.\n");
}


static void reset()
{
	dQuaternion q;
	dQSetIdentity(q);
	dBodySetPosition(mbody,0,0,0+H);
	dBodySetQuaternion(mbody, q);
	dBodySetLinearVel(mbody, 0,0,0);
	dBodySetAngularVel(mbody, 0,0,0);
	dBodyEnable(mbody);
	for ( int i=0; i<halton_numc; ++i )
	{
		dBodyID body = hbody[i];
		if ( !body ) continue;
		dBodySetPosition(body, halton_pos[i][0], halton_pos[i][1], halton_pos[i][2]+H);
		dBodySetQuaternion(body, q);
		dBodySetLinearVel(body, 0,0,0);
		dBodySetAngularVel(body, 0,0,0);
		dBodyEnable(body);
	}
}


// called when a key pressed

static void command(int cmd)
{
	switch (cmd) 
	{
		case ' ':
			reset();
			break;
		default:
			break;
	}
}

static void draw_plane_normals( int nr )
{
	dGeomID geom = hgeom[nr];
	dBodyID body = hbody[nr];
	const dReal *pos = dGeomGetPosition(geom);
	const dReal *rot = dGeomGetRotation(geom);
	const int numf = halton_numf[nr];
	const unsigned int* reader = halton_faces[nr];
	const dReal* verts = halton_verts[nr];
	for ( int f=0; f<numf; ++f )
	{
		const unsigned int sz = *reader++;
		dsSetColor(0.0,0.0,0.4);
		dVector3 xv0,xv1;
		dVector3 c = {0,0,0};
		for ( int j=0; j<sz-1; ++j )
		{
			const unsigned int i0 = reader[j+0];
			const unsigned int i1 = reader[j+1];
			const dReal* v0 = verts+3*i0;
			const dReal* v1 = verts+3*i1;
			//dsDrawLine(v0,v1);
			dBodyGetRelPointPos(body, v0[0],v0[1],v0[2], xv0);
			dBodyGetRelPointPos(body, v1[0],v1[1],v1[2], xv1);
			dsDrawLine( xv0, xv1 );
			if ( j==0 )
				dAddVectors3( c,v0,c );
			dAddVectors3( c,v1,c );
		}
		c[0] = c[0] / sz;
		c[1] = c[1] / sz;
		c[2] = c[2] / sz;
		dVector3 xc;
		dBodyGetRelPointPos(body, c[0],c[1],c[2], xc);

		const dReal* plane = halton_planes[nr] + f*4;
		const dReal w0 = plane[3];
		const dReal w1 = w0 + 0.1;
		dVector3 p0 = { plane[0]*w0, plane[1]*w0, plane[2]*w0 };
		dVector3 p1 = { plane[0]*w1, plane[1]*w1, plane[2]*w1 };
		dVector3 xp0, xp1;
		dBodyGetRelPointPos(body, p0[0],p0[1],p0[2], xp0);
		dBodyGetRelPointPos(body, p1[0],p1[1],p1[2], xp1);
		dVector3 diff;
		dSubtractVectors3( diff, xp1, xp0 );
		dVector3 d;
		dAddVectors3( d, xc, diff );
		dsSetColor(0.2,0.7,0.2);
		dsDrawLine( xc, d );

		reader += sz;
	}
}


static void simLoop(int pause)
{
	double simstep = 1/240.0;
	double dt = dsElapsedTime();

	int nrofsteps = (int) ceilf(dt/simstep);
	nrofsteps = nrofsteps > 8 ? 8 : nrofsteps;

	for (int i=0; i<nrofsteps && !pause; i++)
	{
		dSpaceCollide (space,0,&nearCallback);
		dWorldQuickStep (world, simstep);
		dJointGroupEmpty (contactgroup);
	}

	// Draw the convex objects.
	for ( int i=0; i<halton_numc; ++i )
	{

		dGeomID geom = hgeom[i];
		if ( !geom ) continue;

		draw_plane_normals( i );
#if 1
		dBodyID body = dGeomGetBody(geom);
		const dReal *pos = dGeomGetPosition(geom);
		const dReal *rot = dGeomGetRotation(geom);
		if ( i==75 )
			dsSetColorAlpha(1,1,0,0.39);
		else
			dsSetColorAlpha(1,1,1,0.39);
		dsDrawConvex
		(
			pos, rot,
			halton_planes[i],
			halton_numf[i],
			halton_verts[i],
			halton_numv[i],
			halton_faces[i]
		);
#endif
	}

	if (drawpos)
	{
		dsSetColor(1,0,0.2);
		dsSetTexture(DS_NONE);
		const dReal l = 0.35;
		for ( int i=0; i<halton_numc; ++i )
		{
			dBodyID body = hbody[i];
			if ( !body ) continue;
			const dReal *pos = dBodyGetPosition(body);
			dReal x0[3] = { pos[0]-l, pos[1], pos[2] };
			dReal x1[3] = { pos[0]+l, pos[1], pos[2] };
			dReal y0[3] = { pos[0], pos[1]-l, pos[2] };
			dReal y1[3] = { pos[0], pos[1]+l, pos[2] };
			dReal z0[3] = { pos[0], pos[1], pos[2]-l };
			dReal z1[3] = { pos[0], pos[1], pos[2]+l };
			dsDrawLine(x0,x1);
			dsDrawLine(y0,y1);
			dsDrawLine(z0,z1);
		}
	}

	//draw_plane_normals( 75 );
}


// Check to see if the vertices of the face actually form the same surface normal as was defined for the plane.
static void sanity_check( int nr )
{
	const dReal* verts = halton_verts[nr];
	const dReal* planes = halton_planes[nr];
	const unsigned int* faces = halton_faces[nr];
	const int numf = halton_numf[nr];

	const unsigned int* reader = faces;
	for ( int f=0; f<numf; ++f )
	{
		const dReal* plane = planes + f*4;
		const int sz = *reader++;
		fprintf( stdout, "Volume %d, face %d of size %d at dist %f\n", nr, f, sz, plane[3] );
		for ( int t=0; t<sz-2; ++t )
		{
			const int i0 = reader[  0];
			const int i1 = reader[t+1];
			const int i2 = reader[t+2];
			assert( i1 != i0 );
			assert( i2 != i0 );
			assert( i1 != i2 );
			const dReal* v0 = verts + i0*3;
			const dReal* v1 = verts + i1*3;
			const dReal* v2 = verts + i2*3;
			dVector3 u,v;
		       	dSubtractVectors3( u, v1, v0 );
			dSubtractVectors3( v, v2, v1 );
			dVector3 cross;
			dCalcVectorCross3( cross, u, v );
			dNormalize3( cross );
			const dReal dot = dCalcVectorDot3( plane, cross );
			assert( dot >= 0.999f && dot <= 1.001f );
			const dReal dist0 = dCalcVectorDot3( plane, v0 );
			const dReal dist1 = dCalcVectorDot3( plane, v1 );
			const dReal dist2 = dCalcVectorDot3( plane, v2 );
			const dReal delta0 = dist0 - plane[3];
			const dReal delta1 = dist1 - plane[3];
			const dReal delta2 = dist2 - plane[3];
			assert( fabs(delta0) < 0.0001 );
			assert( fabs(delta1) < 0.0001 );
			assert( fabs(delta2) < 0.0001 );
		}
		reader += sz;
	}
}


int main (int argc, char **argv)
{
#if defined(linux)
	feenableexcept( FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW );
#endif

	dMass m;

	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

	// create world
	dInitODE2(0);
	world = dWorldCreate();
	space = dHashSpaceCreate (0);
	dHashSpaceSetLevels(space, -3, 5);
	dCreatePlane(space,0,0,1,0);	// Add a ground plane.

	contactgroup = dJointGroupCreate (0);
	dWorldSetGravity(world,0,0,-9.8);
	dWorldSetQuickStepNumIterations(world, 32);
	dWorldSetContactMaxCorrectingVel(world, 40);
	dWorldSetMaxAngularSpeed(world, 62.8);
	dWorldSetERP(world, 0.7);
	dWorldSetQuickStepW(world, 0.75); // For increased stability.

	dWorldSetAutoDisableFlag( world, true );
	dWorldSetAutoDisableLinearThreshold( world, 0.01 );
	dWorldSetAutoDisableAngularThreshold( world, 0.03 );
	dWorldSetAutoDisableTime( world, 0.15f );

	const float kernelrad = 0.7;

	mbody = dBodyCreate(world);
	dBodySetPosition(mbody, 0,0,0+H);
	dMassSetSphere( &m, 5, kernelrad );
	dBodySetMass( mbody, &m );

	for (int i=0; i<halton_numc; ++i )
	{
		//if ( i!= 75 ) continue;
		sanity_check( i );
		dGeomID geom = dCreateConvex
		(
			space,
			halton_planes[i],
			halton_numf[i],
			halton_verts[i],
			halton_numv[i],
			halton_faces[i]
		);
		hgeom[i] = geom;
		const dReal x = halton_pos[i][0];
		const dReal y = halton_pos[i][1];
		const dReal z = halton_pos[i][2];
		const dReal dsqr = x*x + y*y + z*z;

		if ( dsqr < kernelrad*kernelrad && solidkernel )
		{
			dGeomSetBody(geom, mbody);
			dGeomSetOffsetPosition(geom, x,y,z);
		}
		else
		{
			dBodyID body = dBodyCreate(world);
			hbody[i] = body;
			dBodySetPosition(body, x,y,z+H);
			dReal volu = halton_volu[i];
			dReal rad = pow( volu * 3 / (4*M_PI), (1/3.0) );
			dMassSetSphere( &m,5,rad );
			dBodySetMass( body,&m );
#if 1
			dBodySetLinearDamping (body, 0.0005);
			dBodySetAngularDamping(body, 0.0300);
#endif

			dGeomSetBody(geom,body);
		}
	}

	// run simulation
	const int w=1920;
	const int h=1080;
	dsSimulationLoop (argc,argv,w,h,&fn);

	dJointGroupEmpty (contactgroup);
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);
	dCloseODE();
	return 0;
}


