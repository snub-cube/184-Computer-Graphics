#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
      //final interpolated vector
      if (points.size() == 1) {
          return points;
      }
      //vector containing intermediate points
      std::vector<Vector2D> pts_vector;
      int idx = 0;
      while (idx < points.size() - 1) {
          //lerp
          Vector2D pt = (1.0f - t) * points[idx] + t * points[idx + 1];
          //append to vector
          pts_vector.push_back(pt);
          idx++;
      }
      return pts_vector;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      //final interpolated vector
      if (points.size() == 1) {
          return points;
      }
      //vector containing intermediate points
      std::vector<Vector3D> pts_vector;
      int idx = 0;
      while (idx < points.size() - 1) {
          //lerp
          Vector3D pt = (1.0f - t) * points[idx] + t * points[idx + 1];
          //append to vector
          pts_vector.push_back(pt);
          idx++;
      }
      return pts_vector;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      vector<Vector3D> pts_vector = points;
      while (pts_vector.size() != 1) { //check final evaluated point
          pts_vector = evaluateStep(pts_vector, t);
      }
      return pts_vector[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    vector<Vector3D> final_u; //store final point of each row 
    int idx = 0;
    while (idx < controlPoints.size()) {
        Vector3D row_final = evaluate1D(controlPoints[idx], u); //get final point of a row
        final_u.push_back(row_final);
        idx++;
    }
    Vector3D final_uv = evaluate1D(final_u, v);//final_uv lies on Bezier surface at the given parameter u and v
    return final_uv;
  }

  //helper. return triangle face area;
  //area =(OA X OB + OB X OC + OC X OA)/2; 
  //let O = A ,so area =(AB X AC)/2;
  double get_area(HalfedgeCIter h) {
      Vector3D va = h->vertex()->position;
      Vector3D vb = h->next()->vertex()->position;
      Vector3D vc = h->next()->next()->vertex()->position;
      Vector3D A = cross(vb - va, vc - va) / 2;
      return A.norm();
  }
  Vector3D Vertex::normal(void) const
  {
      // TODO Part 3.
      // Returns an approximate unit normal at this vertex, computed by
      // taking the area-weighted average of the normals of neighboring
      // triangles, then normalizing.

      HalfedgeCIter h = this->halfedge();
      Vector3D sum_normal(0, 0, 0);
      do {
          sum_normal += get_area(h);
          h = h->twin()->next();
      } while (h != this->halfedge());

      sum_normal.normalize();
      return sum_normal;
  }

  EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0)
  {
      // TODO Part 4.
      // This method should flip the given edge and return an iterator to the flipped edge.
      if (e0->isBoundary()) {
          return e0;
      }

      /* befor flip:  v0
                   h1/ | \h4
                    /h0|  \               f0 | f1
                v1  \  |h5/ v3
                   h2\ | /h3
                       v2
      */
      //define halfedge
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();

      HalfedgeIter h5 = h0->twin();
      HalfedgeIter h3 = h5->next();
      HalfedgeIter h4 = h3->next();

      //define edge, e0 = h0 -> edge()
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h3->edge();
      EdgeIter e4 = h4->edge();

      //define vertice
      VertexIter v0 = h1->vertex();
      VertexIter v1 = h2->vertex();
      VertexIter v2 = h3->vertex();
      VertexIter v3 = h4->vertex();

      //define face
      FaceIter f0 = h0->face(); //left
      FaceIter f1 = h5->face(); //right

      /*after flip:     v0
                     h1/  \h4
                      / h0 \       f0
                   v1 ------ v3    --
                      \ h5 /       f1
                     h2\  /h3
                        v2
     */
     //set halfedge,face0 counter clockwise
      h0->setNeighbors(h4, h5, v1, e0, f0); //next,twin,vertex,edge,face
      h4->setNeighbors(h1, h4->twin(), v3, e4, f0);
      h1->setNeighbors(h0, h1->twin(), v0, e1, f0);
      //face0 counter clockwise
      h5->setNeighbors(h2, h0, v3, e0, f1);
      h2->setNeighbors(h3, h2->twin(), v1, e2, f1);
      h3->setNeighbors(h5, h3->twin(), v2, e3, f1);

      //set edge
      e0->halfedge() = h0;

      //set vertice
      v0->halfedge() = h1;
      v1->halfedge() = h2; //h0 is ok
      v2->halfedge() = h3;
      v3->halfedge() = h4;

      //set face
      f0->halfedge() = h0;  //up
      f1->halfedge() = h5;  //down

      return e0;
  }


  VertexIter HalfedgeMesh::splitEdge(EdgeIter e0)
  {
      //TODO Part 5.
      //This method should split the given edge and return an iterator to the newly inserted vertex.
      //The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if (e0->isBoundary()) {
          return e0->halfedge()->vertex();
      }
      /* befor split:      v0
                       h1/ | \h4
                        /h0|  \               f0|f1
                    v1  \  |h5/ v3
                       h2\ | /h3
                           v2
      */
      //define halfedge
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();

      HalfedgeIter h5 = h0->twin();
      HalfedgeIter h3 = h5->next();
      HalfedgeIter h4 = h3->next();

      //define edge, e0 = h0 -> edge()
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h3->edge();
      EdgeIter e4 = h4->edge();

      //define vertice
      VertexIter v0 = h1->vertex();
      VertexIter v1 = h2->vertex();
      VertexIter v2 = h3->vertex();
      VertexIter v3 = h4->vertex();

      //define face
      FaceIter f0 = h0->face(); //left
      FaceIter f1 = h5->face(); //right


      /*after split:         v0
                           /  |  \
                        h1/ h0|h5 \h4
                         / h6 | h11\       f0 | f3
                      v1 ---- m ---- v3    --------
                         \ h7 | h10/       f1 | f2
                        h2\ h8|h9 /h3
                           \  |  /
                             v2
     */
     //add new elements:
     //define new halfedge
      HalfedgeIter h6 = newHalfedge();
      HalfedgeIter h7 = newHalfedge();
      HalfedgeIter h8 = newHalfedge();
      HalfedgeIter h9 = newHalfedge();
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();
      //define new edge
      EdgeIter e6 = newEdge(); //h6,h7
      EdgeIter e8 = newEdge(); //h8,h9
      EdgeIter e10 = newEdge();//h10,h11
      //define new vertex
      VertexIter vm = newVertex();
      //define new face
      FaceIter f2 = newFace(); //bottom right
      FaceIter f3 = newFace(); //top right

      //update all elements (some elments not changed): 
      //set halfedge //face0 couter clockwise
      h0->setNeighbors(h1, h5, vm, e0, f0); //(next,twin,vertex,edge,face)
      h1->setNeighbors(h6, h1->twin(), v0, e1, f0);
      h6->setNeighbors(h0, h7, v1, e6, f0);
      //face1 counter clockwise
      h7->setNeighbors(h2, h6, vm, e6, f1);
      h2->setNeighbors(h8, h2->twin(), v1, e2, f1);
      h8->setNeighbors(h7, h9, v2, e8, f1);
      //face2 counter clockwise
      h9->setNeighbors(h3, h8, vm, e8, f2);
      h3->setNeighbors(h10, h3->twin(), v2, e3, f2);
      h10->setNeighbors(h9, h11, v3, e10, f2);
      //face3 counter clockwise
      h11->setNeighbors(h4, h10, vm, e10, f3);
      h4->setNeighbors(h5, h4->twin(), v3, e4, f3);
      h5->setNeighbors(h11, h0, v0, e0, f3);

      //set vertices
      v0->halfedge() = h1;
      v1->halfedge() = h2;
      v2->halfedge() = h3;
      v3->halfedge() = h4;
      vm->halfedge() = h0;  //h6,h9,h11?
      vm->position = (v0->position + v2->position) / 2.0f; //set vm position to middle of old e0  

      //set edges
      e0->halfedge() = h0;
      e1->halfedge() = h1;
      e2->halfedge() = h2;
      e3->halfedge() = h3;
      e4->halfedge() = h4;
      e6->halfedge() = h6;
      e8->halfedge() = h8;
      e10->halfedge() = h10;

      //set face
      f0->halfedge() = h0;
      f1->halfedge() = h7;
      f2->halfedge() = h9;
      f3->halfedge() = h11;

      return vm;
  }



  void MeshResampler::upsample(HalfedgeMesh& mesh)
  {
      // TODO Part 6.
      // This routine should increase the number of triangles in the mesh using Loop subdivision.
      // One possible solution is to break up the method as listed below.

      // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
      // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
      // a vertex of the original mesh.
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          //set old vertex isNew false
          v->isNew = false;

          HalfedgeIter h = v->halfedge();
          int n = (int)v->degree();
          float u = (n == 3) ? (3.0f / 16.0f) : (3.0f / (8.0f * n));
          Vector3D original_position = v->position;
          Vector3D original_neighbor_position_sum = Vector3D(0, 0, 0);
          do {
              original_neighbor_position_sum += h->next()->vertex()->position;
              h = h->twin()->next();
          } while (h != v->halfedge());
          //new position for old vertex
          v->newPosition = (1.0f - n * u) * original_position + u * original_neighbor_position_sum;
      }

      // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          //set old edge isNew false
          e->isNew = false;

          HalfedgeCIter h = e->halfedge();
          //vertex position
          Vector3D A = h->vertex()->position;
          Vector3D B = h->twin()->vertex()->position;
          Vector3D C = h->next()->next()->vertex()->position;
          Vector3D D = h->twin()->next()->next()->vertex()->position;
          //new position for new vertex when split
          e->newPosition = 3.0f / 8.0f * (A + B) + 1.0f / 8.0f * (C + D);
      }

      // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
      // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
      // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
      // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

      EdgeIter e = mesh.edgesBegin();
      long old_edge_size = mesh.nEdges();  //loop will add new edge at end.get old edge number first
      for (long couter = 0; couter < old_edge_size; couter++, e++) {
          VertexIter v_new = mesh.splitEdge(e);
          // set isNew for new vertex
          v_new->isNew = true;
          v_new->newPosition = e->newPosition; //assign newPosition to new vertex

          // set isNew for new edge. add tree edge element, but two set to new edge
          HalfedgeIter h = v_new->halfedge();
          h->edge()->isNew = false;   //old element,set false
          h->twin()->next()->edge()->isNew = true; //new element, set true
          h->twin()->next()->twin()->next()->edge()->isNew = false; //new element, old position, set false
          h->twin()->next()->twin()->next()->twin()->next()->edge()->isNew = true; //new element, set true
      }

      // 4. Flip any new edge that connects an old and new vertex.
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          if (e->isNew) {
              VertexIter v0 = e->halfedge()->vertex();
              VertexIter v1 = e->halfedge()->twin()->vertex();
              if (v0->isNew ^ v1->isNew) {
                  mesh.flipEdge(e);
              }
          }
      }

      // 5. Copy the new vertex positions into final Vertex::position.
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->position = v->newPosition;
      }
  }

}