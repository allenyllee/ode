#include <UnitTest++.h>
#include <ode/ode.h>
#include "common.h"

TEST(test_collision_trimesh_sphere_exact)
{
    /*
     * This tests some extreme cases, where a sphere barely touches some triangles
     * with zero depth.
     */
    
    #ifdef dTRIMESH_GIMPACT
    /*
     * Although GIMPACT is algorithmically able to handle this extreme case,
     * the numerical approximation used for the square root produces inexact results.
     */
    return;
    #endif

    {
        const int VertexCount = 4;
        const int IndexCount = 2*3;
        // this is a square on the XY plane
        /*
           3    2
           +----+
           |   /|
           |  / |
           | /  |
           |/   |
           +----+
           0    1
         */
        float vertices[VertexCount * 3] = {
            -1,-1,0,
            1,-1,0,
            1,1,0,
            -1,1,0
        };
        dTriIndex indices[IndexCount] = {
            0,1,2,
            0,2,3
        };
        
        dTriMeshDataID data = dGeomTriMeshDataCreate();
        dGeomTriMeshDataBuildSingle(data,
                                    vertices,
                                    3 * sizeof(float),
                                    VertexCount,
                                    indices,
                                    IndexCount,
                                    3 * sizeof(dTriIndex));
        dGeomID trimesh = dCreateTriMesh(0, data, 0, 0, 0);
        const dReal radius = 4;
        dGeomID sphere = dCreateSphere(0, radius);
        dContactGeom cg[4];
        int nc;
        dVector3 trinormal = { 0, 0, -1 };

        // Test case: sphere touches the diagonal edge
        dGeomSetPosition(sphere, 0,0,radius);
        nc = dCollide(trimesh, sphere, 4, &cg[0], sizeof cg[0]);
        CHECK_EQUAL(2, nc);
        for (int i=0; i<nc; ++i) {
            CHECK_EQUAL(0, cg[i].depth);
            CHECK_ARRAY_EQUAL(trinormal, cg[i].normal, 3);
        }
        
        // now translate both geoms
        dGeomSetPosition(trimesh, 10,30,40);
        dGeomSetPosition(sphere, 10,30,40+radius);
        // check extreme case, again
        nc = dCollide(trimesh, sphere, 4, &cg[0], sizeof cg[0]);
        CHECK_EQUAL(2, nc);
        for (int i=0; i<nc; ++i) {
            CHECK_EQUAL(0, cg[i].depth);
            CHECK_ARRAY_EQUAL(trinormal, cg[i].normal, 3);
        }
        
        // and now, let's rotate the trimesh, 90 degrees on X
        dMatrix3 rot = { 1, 0, 0, 0,
                         0, 0, -1, 0,
                         0, 1, 0, 0 };
        dGeomSetPosition(trimesh, 10,30,40);
        dGeomSetRotation(trimesh, rot);
        
        dGeomSetPosition(sphere, 10,30-radius,40);
        // check extreme case, again
        nc = dCollide(trimesh, sphere, 4, &cg[0], sizeof cg[0]);
        CHECK_EQUAL(2, nc);
        dVector3 rtrinormal = { 0, 1, 0 };
        for (int i=0; i<nc; ++i) {
            CHECK_EQUAL(0, cg[i].depth);
            CHECK_ARRAY_EQUAL(rtrinormal, cg[i].normal, 3);
        }
    }
}



TEST(test_collision_heightfield_ray_fail)
{
    /*
     * This test demonstrated a bug in the AABB handling of the
     * heightfield.
     */
    {
        // Create quick heightfield with dummy data
        dHeightfieldDataID heightfieldData = dGeomHeightfieldDataCreate();
        unsigned char dataBuffer[16+1] = "1234567890123456";
        dGeomHeightfieldDataBuildByte(heightfieldData, dataBuffer, 0, 4, 4, 4, 4, 1, 0, 0, 0);
        dGeomHeightfieldDataSetBounds(heightfieldData, '0', '9');
	    dGeomID height = dCreateHeightfield(0, heightfieldData, 1);

        // Create ray outside bounds
        dGeomID ray = dCreateRay(0, 20);
        dGeomRaySet(ray, 5, 10, 1, 0, -1, 0);
        dContact contactBuf[10];

        // Make sure it does not crash!
        dCollide(ray, height, 10, &(contactBuf[0].geom), sizeof(dContact));

        dGeomDestroy(height);
        dGeomDestroy(ray);
        dGeomHeightfieldDataDestroy(heightfieldData);
    }
}

#include "../ode/demo/convex_prism.h"

TEST(test_collision_ray_convex)
{
    /*
     * Issue 55: ray vs convex collider does not consider the position of the convex geometry.
     */
    {
		dContact contact{};

        // Create convex
	    dGeomID convex = dCreateConvex(0, 
            prism_planes, 
            prism_planecount, 
            prism_points, 
            prism_pointcount,
            prism_polygons);
        dGeomSetPosition(convex,0,0,0);

        // Create ray
        dGeomID ray = dCreateRay(0, 20);

        dGeomRaySet(ray, 0, -10, 0, 0, 1, 0);

        int count = dCollide(ray, convex, 1, &contact.geom, sizeof(dContact));

		CHECK_EQUAL(1,count);
		CHECK_CLOSE(0.0,contact.geom.pos[0], dEpsilon);
		CHECK_CLOSE(-1.0,contact.geom.pos[1], dEpsilon);
		CHECK_CLOSE(0.0,contact.geom.pos[2], dEpsilon);
		CHECK_CLOSE(0.0, contact.geom.normal[0], dEpsilon);
		CHECK_CLOSE(-1.0, contact.geom.normal[1], dEpsilon);
		CHECK_CLOSE(0.0, contact.geom.normal[2], dEpsilon);
		CHECK_CLOSE(9.0, contact.geom.depth, dEpsilon);

		// Move Ray
		dGeomRaySet(ray, 5, -10, 0, 0, 1, 0);

		count = dCollide(ray, convex, 1, &contact.geom, sizeof(dContact));

		CHECK_EQUAL(1,count);
		CHECK_CLOSE(5.0, contact.geom.pos[0], dEpsilon);
		CHECK_CLOSE(-1.0, contact.geom.pos[1], dEpsilon);
		CHECK_CLOSE(0.0, contact.geom.pos[2], dEpsilon);
		CHECK_CLOSE(0.0, contact.geom.normal[0], dEpsilon);
		CHECK_CLOSE(-1.0, contact.geom.normal[1], dEpsilon);
		CHECK_CLOSE(0.0, contact.geom.normal[2], dEpsilon);
		CHECK_CLOSE(9.0, contact.geom.depth, dEpsilon);

		// Rotate Convex
		dMatrix3 rotate90z
		{
			0,-1,0,0,
			1,0,0,0,
			0,0,1,0 
		};
		dGeomSetRotation(convex, rotate90z);

		count = dCollide(ray, convex, 1, &contact.geom, sizeof(dContact));

		CHECK_EQUAL(0,count);

		// Move Ray
		dGeomRaySet(ray, 10, 0, 0, -1, 0, 0);
		count = dCollide(ray, convex, 1, &contact.geom, sizeof(dContact));

		CHECK_EQUAL(1,count);
		CHECK_CLOSE(1.0, contact.geom.pos[0], dEpsilon);
		CHECK_CLOSE(0.0, contact.geom.pos[1], dEpsilon);
		CHECK_CLOSE(0.0, contact.geom.pos[2], dEpsilon);
		CHECK_CLOSE(1.0, contact.geom.normal[0], dEpsilon);
		CHECK_CLOSE(0.0, contact.geom.normal[1], dEpsilon);
		CHECK_CLOSE(0.0, contact.geom.normal[2], dEpsilon);
		CHECK_CLOSE(9.0,contact.geom.depth, dEpsilon);


		// Move Ray
		dGeomRaySet(ray, 10, 1000, 1000, -1, 0, 0);
		// Move Geom
		dGeomSetPosition(convex, 0, 1000, 1000);

		count = dCollide(ray, convex, 1, &contact.geom, sizeof(dContact));

		CHECK_EQUAL(1, count);
		CHECK_CLOSE(1.0, contact.geom.pos[0], dEpsilon);
		CHECK_CLOSE(1000.0, contact.geom.pos[1], dEpsilon);
		CHECK_CLOSE(1000.0, contact.geom.pos[2], dEpsilon);
		CHECK_CLOSE(1.0, contact.geom.normal[0], dEpsilon);
		CHECK_CLOSE(0.0, contact.geom.normal[1], dEpsilon);
		CHECK_CLOSE(0.0, contact.geom.normal[2], dEpsilon);
		CHECK_CLOSE(9.0, contact.geom.depth, dEpsilon);

		dGeomDestroy(convex);
        dGeomDestroy(ray);
    }
}
