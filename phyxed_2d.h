
/*
phyxed_2d.h - v0.01 - public domain - Anders Elfgren @srekel, 2017

# Phyxed 2D

A simple 2d physics library with fixed-point support

See github for latest version: https://github.com/Srekel/phyxed-2d

## Usage

In *ONE* source file, put:

```C
#define PHYXED_IMPLEMENTATION

// Define any of these if you wish to override them.
// (There are more. Find them in the beginning of the code.)
#define PHYXED_assert
#define PHYXED_memcpy

#include "phyxed_2d.h"
```

Other source files should just include phyxed_2d.h

## Notes

See the accompanying SDL demo and unit test projects for references on how to
use it. Or the documentation on the github page.

## References and relevant or interesting physics links

1. http://www.dyn4j.org/2010/01/sat/
2. http://www.dyn4j.org/2011/11/contact-points-using-clipping/
3. https://code.google.com/archive/p/box2d/downloads
4. https://gamedevelopment.tutsplus.com/series/how-to-create-a-custom-physics-engine--gamedev-12715
5. http://rasmusbarr.github.io/blog/dod-physics.html
6. https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/
7. http://www.codercorner.com/blog/?p=1207
8. https://github.com/kroitor/gjk.c
9. https://www.cs.cmu.edu/~baraff/sigcourse/notesf.pdf
10. http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/

## License

Basically Public Domain / MIT.
See end of file for license information.

*/

#ifndef INCLUDE_PHYXED_LIBRARY_H
#define INCLUDE_PHYXED_LIBRARY_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PHYXED_NO_STDINT
#include <stdint.h>
#endif

#ifndef PhyxedNumber
#define PhyxedNumber float
#endif

#ifndef PhyxedNumber
#define PhyxedBodyId uint16_t
#endif

typedef struct PhyxedVector2 {
    PhyxedNumber x;
    PhyxedNumber y;
} PhyxedVector2;

typedef struct PhyxedTransform {
    PhyxedVector2 pos;
    PhyxedNumber  rot;
} PhyxedTransform;

typedef struct PhyxedMatrix {
    union {
        struct {
            PhyxedNumber m00, m01;
            PhyxedNumber m10, m11;
        };
        struct {
            PhyxedVector2 row1;
            PhyxedVector2 row2;
        };
    };
} PhyxedMatrix;

typedef struct PhyxedBody {
    PhyxedNumber inertia_inv;
    PhyxedNumber mass_inv;
    PhyxedNumber friction;
} PhyxedBody;

typedef struct PhyxedBodyTransform {
    PhyxedTransform transform;
    PhyxedBodyId    body_id;
} PhyxedBodyTransform;

typedef struct PhyxedBodyPair {
    PhyxedBodyId first;
    PhyxedBodyId second;
} PhyxedBodyPair;

typedef struct PhyxedBodyMomentum {
    PhyxedVector2 vel;
    PhyxedNumber  vel_rot;
} PhyxedBodyMomentum;

// clang-format off
typedef struct PhyxedBoxShape {
	PhyxedVector2 extents;
} PhyxedBox;
// clang-format on

typedef struct PhyxedCollision {
    PhyxedVector2 pos;
    PhyxedNumber  penetration;
    PhyxedVector2 normal;
    PhyxedNumber  friction;
} PhyxedCollision;

typedef struct PhyxedCollisionInput {
    PhyxedBodyTransform* transforms;
    PhyxedBody*          bodies;
    PhyxedBodyMomentum*  body_momentums;
    uint32_t             num_collisions;
    uint32_t             collision_capacity;

    struct {
        PhyxedBoxShape*      shapes;
        PhyxedBodyTransform* transforms;
        uint32_t*            num_shapes;
    } boxes;

    void*    temp_buffer;
    uint32_t temp_buffer_capacity;
} PhyxedCollisionInput;

typedef struct PhyxedCollisionOutput {
    PhyxedCollision* collisions;
    PhyxedBodyPair*  body_pairs;
    uint32_t         num_collisions;
    const uint32_t   collision_capacity;
} PhyxedCollisionOutput;

typedef struct PhyxedCollisionBroadphaseOutput {
    PhyxedBodyPair* body_pairs;
    uint32_t        num_collisions_box_vs_box;
    uint32_t        collision_capacity;
} PhyxedCollisionBroadphaseOutput;

#ifdef __cplusplus
}
#endif

#ifdef PHYXED_IMPLEMENTATION

#ifndef PHYXED_assert
#include <assert.h>
#define PHYXED_assert assert;
#endif

#ifndef PHYXED_memcpy
#include <string.h>
#define PHYXED_memcpy memcpy
#endif

#ifndef PHYXED_memset
#include <string.h>
#define PHYXED_memset memset
#endif

#ifndef PHYXED_min
#define PHYXED_min( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

#ifndef PHYXED_max
#define PHYXED_max( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#endif

#ifndef __cplusplus
#include <stdbool.h>
#endif

void
phyxed__normalize( PhyxedVector2* vec ) {
    PhyxedNumber length = PHYXED_sqrt( vec->x * vec->x + vec->y * vec->y );
    vec->x              = PHYXED_div( vec->x, length );
    vec->y              = PHYXED_div( vec->y, length );
}

void
phyxed__collide_broadphase( PhyxedCollisionBroadphaseOutput* broadphase_collisions,
                            const PhyxedCollisionInput*      input ) {
    {
        // Box vs box
        const PhyxedNumber fudge = 2; // Probably sqrt(2) but eh.
        for ( uint32_t first = 0; first < input->boxes.num_shapes; ++first ) {
            const PhyxedBoxShape* shape1 = input->boxes.shapes[first];
            PhyxedNumber max_radius1 = PHYXED_max( shape1->extents.x, shape1->extents.x ) * fudge;
            const PhyxedTransform* transform1 = input->boxes.transforms[first];

            if ( output->collision_capacity < output->num_collisions + input->boxes.num_shapes ) {
                // Too small buffer!
                // TODO: Mark this somehow
                break;
            }
            for ( uint32_t second = first + 1; second < input->boxes.num_shapes; ++second ) {
                const PhyxedBoxShape* shape2 = input->boxes.shapes[second];
                PhyxedNumber          max_radius2 =
                  PHYXED_max( shape2->extents.x, shape2->extents.x ) * fudge;
                const PhyxedTransform* transform2 = input->boxes.transforms[second];

                // TODO: Masking and multiple shapes per body (transform hierarchy)

                if ( transform1->pos.x - transform2->pos.x < max_radius1 + max_radius2 ||
                     transform1->pos.y - transform2->pos.y < max_radius1 + max_radius2 ) {
                    PhyxedBodyPair* pair = output->body_pairs[output->num_collisions_box_vs_box++];
                    pair->first          = first;
                    pair->second         = second;
                    // TODO might need to store more
                }
            }
        }
    }
}

void
phyxed__generate_vertices_box( PhyxedBoxShape*  shape,
                               PhyxedTransform* transform,
                               PhyxedVector2*   vertices_out ) {
    // Top left, top right, bottom right, bottom left, CW
    PhyxedVector2 vertices[] = {
        { PHYXED_sub( transform->pos.x,
                      PHYXED_mul( shape->extents.x, PHYXED_cos( transform->rot ) ) ),
          PHYXED_add( transform->pos.y,
                      PHYXED_mul( shape->extents.y, PHYXED_cos( transform->rot ) ) ) },
        { PHYXED_add( transform->pos.x,
                      PHYXED_mul( shape->extents.x, PHYXED_cos( transform->rot ) ) ),
          PHYXED_add( transform->pos.y,
                      PHYXED_mul( shape->extents.y, PHYXED_cos( transform->rot ) ) ) },
        { PHYXED_add( transform->pos.x,
                      PHYXED_mul( shape->extents.x, PHYXED_cos( transform->rot ) ) ),
          PHYXED_sub( transform->pos.y,
                      PHYXED_mul( shape->extents.y, PHYXED_cos( transform->rot ) ) ) },
        { PHYXED_sub( transform->pos.x,
                      PHYXED_mul( shape->extents.x, PHYXED_cos( transform->rot ) ) ),
          PHYXED_sub( transform->pos.y,
                      PHYXED_mul( shape->extents.y, PHYXED_cos( transform->rot ) ) ) }
    };

    PHYXED_memcpy( vertices_out, &vertices, sizeof( vertices ) );
}

bool
phyxed__generate_axes( PhyxedVector2* vertices, int num_vertices, PhyxedVector2* axes_out ) {
    for ( int i = 0; i < num_vertices; ++i ) {
        int next_vertex_index = i < num_vertices ? i + 1 : 0;
        axis_out[i].x         = PHYXED_sub( vertices[next_vertex_index].x, vertices1[i].x );
        axis_out[i].y         = PHYXED_sub( vertices[next_vertex_index].y, vertices1[i].y );
        phyxed__normalize( &axis_out[i] );
    }
}

void
phyxed__project_onto_axis( PhyxedVector2* axis,
                           PhyxedVector2* vertices,
                           int            num_vertices,
                           PhyxedNumber*  min_out,
                           PhyxedNumber*  max_out ) {

    PhyxedNumber min = phyxed__vec2_dot( &axis, &vertices[0] );
    PhyxedNumber max = min;
    for ( int i = 1; i < num_vertices; ++i ) {
        PhyxedNumber projection = phyxed__vec2_dot( axis, &vertices[i] );
        if ( projection < min ) {
            min = projection;
        }
        else if ( projection > max ) {
            max = projection;
        }
    }

    *min_out = min;
    *max_out = max;
    int a    = 3;
}

bool
phyxed__overlap( PhyxedVector2* axes,
                 PhyxedVector2* vertices1,
                 PhyxedVector2* vertices2,
                 int            num_vertices,
                 int*           axis_out,
                 PhyxedNumber*  overlap_out ) {

    PhyxedNumber smallest_overlap = 100000; // TODO
    int          smallest_axis    = 0;
    for ( int i = 0; i < num_vertices; ++i ) {
        PhyxedNumber min1, max1;
        PhyxedNumber min2, max2;
        phyxed__project_onto_axis( &axes[i], &vertices1, num_vertices, &min1, &max1 );
        phyxed__project_onto_axis( &axes[i], &vertices2, num_vertices, &min2, &max2 );
        PhyxedNumber overlap = 0;
        if ( !( min1_1 <= min1_2 && min1_2 <= max1_1 ) ) {
            return false;
        }
        else {
            PhyxedNumber overlap = PHYXED_min( PHYXED_sub( min2, min1 ), PHYXED_sub( max1, min2 ) );
            if ( overlap < smallest_overlap ) {
                smallest_overlap = overlap;
                smallest_axis    = i;
            }
        }
    }

    *axis_out    = smallest_axis;
    *overlap_out = smallest_overlap;
    return true;
}

PhyxedNumber
phyxed__abs( PhyxedNumber scalar ) {
    return scalar >= 0 ? scalar : PHYXED_mul(scalar, PHYXED_minus_one));
}

PhyxedNumber
phyxed__vec2_dot( PhyxedVector2* v1, PhyxedVector2* v2 ) {
    return PHYXED_add( PHYXED_mul( v1->x, v2->x ), PHYXED_mul( v1->y, v2->y ) );
}

PhyxedVector2
phyxed__vec2_scale( PhyxedVector2* v1, PhyxedNumber scalar ) {
    return { PHYXED_mul( v1->x, scalar ), PHYXED_mul( v1->y, scalar ) };
}

PhyxedVector2
phyxed__vec2_cross( PhyxedVector2* v1 ) {
    return { PHYXED_mul( v1->y, PHYXED_minus_one), v1->x ) };
}

PhyxedVector2
phyxed__vec2_sub( PhyxedVector2* v1, PhyxedVector2* v1 ) {
    return { PHYXED_sub( v1->x, v2->x ), PHYXED_sub( v1->y, v2->y ) };
}

typedef struct PhyxedEdge {
    PhyxedVector2 max;
    PhyxedVector2 v1;
    PhyxedVector2 v2;
} PhyxedEdge;

typedef struct PhyxedClippedPoints {
    PhyxedVector2 points[3];
    PhyxedNumber  depths[3];
    uint8_t       count;
} PhyxedClippedPoints;

typedef struct PhyxedCollisionManifold {
    PhyxedVector2 points[3];
    PhyxedNumber  depths[3];
    uint8_t       count;
} PhyxedClippedPoints;

void
phyxed__find_feature( PhyxedVector2* vertices,
                      int            num_vertices,
                      PhyxedVector2* separation_axis,
                      PhyxedEdge*    out_edge,
                      bool           out_edges_flipped ) {
    // http://www.dyn4j.org/2011/11/contact-points-using-clipping/

    PhyxedNumber max_projection = 0;
    int          index          = 0;
    for ( int        i          = 0; i < num_vertices; ++i ) {
        PhyxedNumber projection = phyxed__vec2_dot( vertices[i], separation_axis );
        if ( projection > max_projection ) {
            max_projection = projection;
            index          = i;
        }
    }

    PhyxedVector2 v      = vertices[index];
    PhyxedVector2 v_prev = vertices[index > 0 ? index - 1 : num_vertices - 1];
    PhyxedVector2 v_next = vertices[index < num_vertices - 1 ? index + 1 : 0];

    PhyxedVector2 left  = v - v_prev;
    PhyxedVector2 right = v_next - v;
    phyxed__normalize( &left );
    phyxed__normalize( &right );
    if ( phyxed__vec2_dot( &right, separation_axis ) <
         phyxed__vec2_dot( &left, separation_axis ) ) {
        out_edge->max = v;
        out_edge->v1  = v;
        out_edge->v2  = v_next;
    }
    else {
        out_edge->max = v;
        out_edge->v1  = v_prev;
        out_edge->v2  = v;
    }
}

PhyxedClippedPoints
phyxed__clip( PhyxedVector2* v1, PhyxedVector2* v2, PhyxedVector2* normal, PhyxedNumber offset ) {
    PhyxedClippedPoints cp;
    cp.count        = 0;
    PhyxedNumber d1 = PHYXED_sub( phyxed__vec2_dot( v1, normal ), offset );
    PhyxedNumber d2 = PHYXED_sub( phyxed__vec2_dot( v2, normal ), offset );
    if ( d1 >= 0 ) {
        cp.points[cp.count++] = *v1;
    }
    if ( d2 >= 0 ) {
        cp.points[cp.count++] = *v2;
    }

    if ( PHYXED_mul( d1, d2 ) < PHYXED_zero ) {
        PhyxedVector2 edge    = phyxed__vec2_sub( v2, v1 );
        PhyxedNumber  u       = PHYXED_div( d1, PHYXED_sub( d1, d2 ) );
        edge                  = phyxed__vec2_scale( &edge, u );
        edge                  = phyxed__vec2_add( &edge, v1 );
        cp.points[cp.count++] = edge;
    }

    return cp;
}

PhyxedClippedPoints
phyxed__clip_features( PhyxedEdge* edge1, PhyxedEdge* edge2, PhyxedVector2* separation_normal ) {
    PhyxedVector2 edge1_dir = phyxed__vec2_sub( edge1.v2, edge1.v1 );
    PhyxedVector2 edge2_dir = phyxed__vec2_sub( edge2.v2, edge2.v1 );
    phyxed__normalize( &edge1_dir );
    phyxed__normalize( &edge2_dir );
    PhyxedEdge*   reference;
    PhyxedEdge*   incident;
    PhyxedVector2 ref_vec;
    PhyxedVector2 inc_vec;
    bool          edges_flipped;
    if ( phyxed__abs( phyxed__vec2_dot( &edge1_dir, separation_normal ) ) <
         phyxed__abs( phyxed__vec2_dot( &edge2_dir, separation_normal ) ) ) {
        reference     = &edge1;
        incident      = &edge2;
        ref_vec       = &edge1_dir;
        inc_vec       = &edge2_dir;
        edges_flipped = false;
    }
    else {
        reference     = &edge2;
        incident      = &edge1;
        ref_vec       = &edge2_dir;
        inc_vec       = &edge1_dir;
        edges_flipped = true;
    }

    PhyxedNumber        o1  = phyxed__vec2_dot( ref_vec, reference->v1 );
    PhyxedClippedPoints cp1 = phyxed__clip( &incident->v1, &incident->v2, ref_vec, o1 );
    if ( cp1.count < 2 ) {
        return;
    }

    PhyxedNumber        o1          = phyxed__vec2_dot( ref_vec, reference->v2 );
    PhyxedVector2       rev_vec_inv = phyxed__vec2_scale( &rev_vec, PHYXED_minus_one );
    PhyxedClippedPoints cp2 = phyxed__clip( &cp1.points[0], &cp1.points[1], &rev_vec_inv, -o2 );
    if ( cp2.count < 2 ) {
        return;
    }

    PhyxedVector2 ref_normal = phyxed__vec2_cross( ref_vec );
    if ( edges_flipped ) {
        ref_normal = phyxed__vec2_scale( &ref_normal, PHYXED_minus_one );
    }

    PhyxedNumber max = phyxed__vec2_dot( ref_normal, reference->max );
    cp2.depths[0]    = PHYXED_SUB( phyxed_dot( &ref_normal, &cp2.points[0] ), max );
    cp2.depths[1] = PHYXED_SUB( phyxed_dot( &ref_normal, &cp2.points[1] ), max );

    if ( cp2.depths[0] < PHYXED_zero ) {
        cp.points[0] = cp.points[1];
        cp.depths[0] = cp.depths[1];
        cp.points[1] = cp.points[2];
        cp.depths[1] = cp.depths[2];
        --cp.count;
    }
    if ( cp2.depths[0] < PHYXED_zero ) {
        cp.points[1] = cp.points[2];
        cp.depths[1] = cp.depths[2];
        --cp.count;
    }

    return cp2;
}

void
phyxed__collide_narrowphase( PhyxedCollisionBroadphaseOutput* output,
                             PhyxedCollisionBroadphaseOutput* broadphase_collisions,
                             const PhyxedCollisionInput*      input ) {

    {
        // Box vs box
        uint32_t num_collisions = broadphase_collisions->num_collisions_box_vs_box;
        for ( uint32_t pair_index = 0; pair_index < num_collisions; ++pair_index ) {
            PhyxedBodyPair*       pair   = broadphase_collisions->body_pairs[pair_index];
            const PhyxedBoxShape* shape1 = input->boxes.shapes[first];
            const PhyxedBoxShape* shape2 = input->boxes.shapes[second];

            const PhyxedTransform* transform1 = shape1->transform;
            const PhyxedTransform* transform2 = shape2->transform;

            PhyxedVector2 vertices1[4];
            PhyxedVector2 vertices2[4];
            phyxed__generate_vertices_box( shape1, transform1, vertices1 );
            phyxed__generate_vertices_box( shape2, transform2, vertices2 );

            // SAT
            // TODO: We only need 2 axis for box
            PhyxedVector2 axes1[4];
            PhyxedVector2 axes2[4];
            phyxed__generate_axes( vertices1, 4, axes1 );
            phyxed__generate_axes( vertices2, 4, axes2 );

            PhyxedNumber smallest_overlap1 = 0;
            int          smallest_axis1    = 0;
            if ( !phyxed__overlap(
                   axis1, vertices1, vertices2, 4, &smallest_axis1, &smallest_overlap1 ) ) {
                continue;
            }

            PhyxedNumber smallest_overlap2 = 0;
            int          smallest_axis2    = 0;
            if ( !phyxed__overlap(
                   axis2, vertices1, vertices2, 4, &smallest_axis2, &smallest_overlap2 ) ) {
                continue;
            }

            // We have a collision, use clipping to calculate collision points / manifold
            PhyxedVector2 separation_normal =
              smallest_overlap1 < smallest_overlap2 ? axis1[smallest_axis1] : axis2[smallest_axis2];
            PhyxedEdge edge1;
            PhyxedEdge edge2;
            phyxed__find_feature( vertices1, &separation_normal, &edge1 );
            PhyxedVector2 edge2_normal = phyxed__vec2_scale( separation_normal, PHYXED_minus_one );
            phyxed__find_feature( vertices1, &edge2_normal, &edge2 );

            PhyxedCollisionManifold manifold =
              phyxed__clip_features( &edge1, &edge2, &separation_normal );

            PhyxedBodyPair* pair_out = output->body_pairs[output->num_collisions];
            *pair_out                = *pair;

            PhyxedCollision* collision = output->collisions[output->num_collisions];
            collision->pos =

              output->_num_collisions++;
        }
    }
}

void
ResolveCollision( Object A, Object B ) {
    // Calculate relative velocity
    Vec2 rv = B.velocity -
              A.velocity

              // Calculate relative velocity in terms of the normal direction
              float velAlongNormal = DotProduct( rv, normal )

      // Do not resolve if velocities are separating
      if ( velAlongNormal > 0 ) return;

    // Calculate restitution
    float e = min( A.restitution, B.restitution )

      // Calculate impulse scalar
      float j = -( 1 + e )* velAlongNormal j /= 1 / A.mass +
                                                1 /
                                                  B.mass

                                                  // Apply impulse
                                                  Vec2 impulse = j* normal A.velocity -=
      1 / A.mass* impulse B.velocity += 1 / B.mass * impulse
}

// const Vec2 gravity( 0, -10.0f )
// velocity += force * (1.0f / mass + gravity) * dt
// angularVelocity += torque * (1.0f / momentOfInertia) * dt
// position += velocity * dt
// orient += angularVelocity * dt

void Body::ApplyImpulse( const Vec2& impulse, const Vec2& contactVector )
{
  velocity += 1.0f / mass * impulse;
  angularVelocity += 1.0f / inertia * Cross( contactVector, impulse );
}

void
phyxed__collide_resolve( PhyxedCollisionBroadphaseOutput* output,
                         PhyxedCollisionBroadphaseOutput* broadphase_collisions,
                         const PhyxedCollisionInput*      input ) {

    float mass_sum = A.mass + B.mass float ratio = A.mass / mass_sum A.velocity -= ratio* impulse

                                                     ratio = B.mass / mass_sum B.velocity +=
      ratio * impulse
}
void
phyxed_collide( PhyxedCollisionOutput* output, const PhyxedCollisionInput* input ) {
    phyxed__collide_broadphase( output, input );
    phyxed__collide_narrowphase( output, input );
    phyxed__collide_resolve( output, input );
}

#endif // PHYXED_IMPLEMENTATION
#endif // INCLUDE_PHYXED_LIBRARY_H

/*
------------------------------------------------------------------------------
This software is available under 2 licenses -- choose whichever you prefer.
------------------------------------------------------------------------------
ALTERNATIVE A - MIT License
Copyright (c) 2017 Anders Elfgren
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
------------------------------------------------------------------------------
ALTERNATIVE B - Public Domain (www.unlicense.org)
This is free and unencumbered software released into the public domain.
Anyone is free to copy, modify, publish, use, compile, sell, or distribute this
software, either in source code form or as a compiled binary, for any purpose,
commercial or non-commercial, and by any means.
In jurisdictions that recognize copyright laws, the author or authors of this
software dedicate any and all copyright interest in the software to the public
domain. We make this dedication for the benefit of the public at large and to
the detriment of our heirs and successors. We intend this dedication to be an
overt act of relinquishment in perpetuity of all present and future rights to
this software under copyright law.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
------------------------------------------------------------------------------
*/
