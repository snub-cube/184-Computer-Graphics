#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // point masses in row-major order
    double grid_w = width / (double)(num_width_points - 1);
    double grid_h = height / (double)(num_height_points - 1);

    for (int j = 0; j < num_height_points; j++) {
       for (int i = 0; i < num_width_points; i++) {
           Vector3D position;
           bool set_pin = false; //set all pinned to false and update next setp
           if (orientation == HORIZONTAL) {
               position.x = i * grid_w;
               position.y = 1.0;
               position.z = j * grid_h;
               PointMass pm = PointMass(position, set_pin);
               point_masses.emplace_back(pm);
           }
           if (orientation == VERTICAL) {
               position.x = i * grid_w;
               position.y = j * grid_h;
               position.z = ((double)rand() / RAND_MAX - 0.5) / 500.0;
               PointMass pm = PointMass(position, set_pin);
               point_masses.emplace_back(pm);
           }
       }
    }

    // update pinned in row-major order
    for (auto a_pined : pinned) {
       int i = a_pined[0];
       int j = a_pined[1];
       if (i < num_width_points && j < num_height_points) {
           point_masses[j * num_width_points + i].pinned = true;
       }
    }

    // creat springs
    for (int j = 0; j < num_height_points; j++) {
       for (int i = 0; i < num_width_points; i++) {
           PointMass* pm_self = &point_masses[j * num_width_points + i];
           //structural spring
           if (i > 0) {
               PointMass* pm_left = &point_masses[j * num_width_points + i - 1];
               springs.emplace_back(Spring(pm_self, pm_left, STRUCTURAL));
           }
           if (j > 0) {
               PointMass* pm_up = &point_masses[(j - 1) * num_width_points + i];
               springs.emplace_back(Spring(pm_self, pm_up, STRUCTURAL));
           }

           //shearing spring
           if (i > 0 && j > 0) {
               PointMass* pm_left_up = &point_masses[(j - 1) * num_width_points + i - 1];
               springs.emplace_back(Spring(pm_self, pm_left_up, SHEARING));
           }
           if (i < num_width_points - 1 && j > 0) {
               PointMass* pm_right_up = &point_masses[(j - 1) * num_width_points + i + 1];
               springs.emplace_back(Spring(pm_self, pm_right_up, SHEARING));
           }

           //bending spring
           if (i > 1) {
               PointMass* pm_left2 = &point_masses[j * num_width_points + i - 2];
               springs.emplace_back(Spring(pm_self, pm_left2, BENDING));
           }
           if (j > 1) {
               PointMass* pm_up2 = &point_masses[(j - 2) * num_width_points + i];
               springs.emplace_back(Spring(pm_self, pm_up2, BENDING));
           }
       }
    }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  for (PointMass& pm : point_masses) { // & ,reference to update pm in point_masses
      pm.forces = 0;
      for (Vector3D a : external_accelerations) {
          pm.forces += mass * a;
      }
  }

  for (Spring& spr : springs) {
        if ((spr.spring_type == STRUCTURAL) && !cp->enable_structural_constraints
            || (spr.spring_type == SHEARING) && !cp->enable_shearing_constraints
            || (spr.spring_type == BENDING) && !cp->enable_bending_constraints) {
            continue;
        }
        Vector3D pa = spr.pm_a->position;
        Vector3D pb = spr.pm_b->position;
        double ks = (spr.spring_type == BENDING) ? cp->ks * 0.2 : cp->ks;

        double fs_mag = ks * ((pb - pa).norm() - spr.rest_length);
        Vector3D fs = fs_mag * (pb - pa).unit();

        spr.pm_a->forces += fs;
        spr.pm_b->forces -= fs;

    }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (PointMass& pm : point_masses) { //& reference to update pm in  point_masses
      if (!pm.pinned) {
          Vector3D a = pm.forces / mass;  //acceleration
          Vector3D temp = pm.position;
          pm.position += (1.0 - cp->damping / 100.0) * (pm.position - pm.last_position) + a * delta_t * delta_t;
          pm.last_position = temp;
      }
  }


  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (PointMass& pm : point_masses) {
      self_collide(pm, simulation_steps);
  }


  // TODO (Part 3): Handle collisions with other primitives.
  for (PointMass& pm : point_masses) {
      for (CollisionObject* obj : *collision_objects) {
          obj->collide(pm);
      }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (Spring& spr : springs) {
      if ((spr.spring_type == STRUCTURAL) && !cp->enable_structural_constraints
          || (spr.spring_type == SHEARING) && !cp->enable_shearing_constraints
          || (spr.spring_type == BENDING) && !cp->enable_bending_constraints) {
          continue;
      }
      Vector3D vec_ab = spr.pm_b->position - spr.pm_a->position;
      double len_excess = vec_ab.norm() - 1.1 * spr.rest_length;

      if (len_excess > 0) {
          Vector3D correction = len_excess * vec_ab.unit();
          if (spr.pm_a->pinned && spr.pm_b->pinned) {
              continue;
          }
          if (!spr.pm_a->pinned && !spr.pm_b->pinned) {
              spr.pm_a->position += 0.5 * correction;
              spr.pm_b->position -= 0.5 * correction;
          }
          if (!spr.pm_a->pinned && spr.pm_b->pinned) {
              spr.pm_a->position += correction;
          }
          if (spr.pm_a->pinned && !spr.pm_b->pinned) {
              spr.pm_b->position -= correction;
          }
      }
  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass& pm : point_masses) {
        float hash_key = hash_position(pm.position);
        if (map.find(hash_key) == map.end()) {
            map[hash_key] = new vector<PointMass*>();
        }
        map[hash_key]->push_back(&pm);
    }

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
    Vector3D correction_sum = Vector3D(0, 0, 0);
    int counter = 0;

    float hash_key = hash_position(pm.position);
    for (PointMass* candidate : *(map[hash_key])) {
        if (candidate == &pm) {
            continue;
        }
        Vector3D direction = candidate->position - pm.position; // vector from pm to candidate
        if (direction.norm() < 2.0 * thickness) {
            correction_sum += (direction.norm() - 2.0 * thickness) * direction.unit();
            counter++;
        }
    }

    if (counter > 0) {
        Vector3D correction = correction_sum / counter;
        pm.position = pm.position + correction / simulation_steps;
    }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  
  double  w = 3.0 * width / num_width_points;
  double  h = 3.0 * height / num_height_points;
  double  t = max(w, h);

  double  x = floor(pos.x / w);
  double  y = floor(pos.y / h);
  double  z = floor(pos.z / t);

  //double hash_key = pow(x, 3) + pow(y, 2) + z;
  double hash_key = pow(113, 3)*x + pow(113, 2)*y + 113*z;
  return (float)hash_key;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
